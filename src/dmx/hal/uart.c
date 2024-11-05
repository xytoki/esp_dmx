#include "include/uart.h"

#include "dmx/hal/include/timer.h"
#include "dmx/include/service.h"
#include "driver/uart.h"
#include "endian.h"
#include "rdm/include/driver.h"
#include "rdm/include/uid.h"

// Include these headers when either logging for debug purposes or toggling GPIO while in the ISR
//#include "esp_log.h"
//#include "soc/gpio_num.h"
//#include "driver/gpio.h"

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include "esp_private/esp_clk.h"
#include "esp_private/periph_ctrl.h"
#include "esp_timer.h"
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
#include "soc/uart_periph.h"
#endif
#else
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#endif

#define DMX_UART_FULL_DEFAULT 1
#define DMX_UART_EMPTY_DEFAULT 8

static struct dmx_uart_t {
  const int num;
  uart_dev_t *const dev;
  intr_handle_t isr_handle;
} dmx_uart_context[DMX_NUM_MAX] = {
    {.num = 0, .dev = UART_LL_GET_HW(0)},
    {.num = 1, .dev = UART_LL_GET_HW(1)},
#if SOC_UART_NUM > 2
    {.num = 2, .dev = UART_LL_GET_HW(2)},
#endif
};

enum {
  RDM_TYPE_IS_NOT_RDM = 0,  // The packet is not RDM.
  RDM_TYPE_IS_DISCOVERY,    // The packet is an RDM discovery request.
  RDM_TYPE_IS_RESPONSE,     // The packet is an RDM response.
  RDM_TYPE_IS_BROADCAST,    // The packet is a non-discovery RDM broadcast.
  RDM_TYPE_IS_REQUEST,      // The packet is a standard RDM request.
  RDM_TYPE_IS_UNKNOWN,      // The packet is RDM, but it is unclear what type it is.
};

// Relatively large refactor of the receive ISR while debugging the issue of inserting and removing the DMX cable
// The problem with inserting and removing the DMX cable is documented in Jira 783
static void DMX_ISR_ATTR dmx_uart_isr(void *arg) 
{
  const int64_t now = dmx_timer_get_micros_since_boot();
  dmx_driver_t *const driver = arg;
  const dmx_port_t dmx_num = driver->dmx_num;
  int task_awoken = false;
  // This variable was created to count the number of byte that are received between the DMX Breaks. This allows the 
  // length to be updated in real time from frame to frame. Previously this code would only update the length if a break was 
  // detected while the state machine was still processing data. However, that was the bug in this software. Because the processing 
  // of data would get out of sync with the flow of data and the length would be incorrectly assigned. Having the length always 
  // be validated ensures it can re-sync after framing errors on the bus
  static uint16_t dmxFrameLengthTracker = 0;

  // For debugging purposes - set GPIO14 LOW when entering the UART ISR
  // gpio_set_level(GPIO_NUM_14, 0);

  // This is an interesting way to handle the UART receive ISR - but we will basically stay in here so long as there is data to process
  while (true) 
  {
    // When the ISR is triggered first obtain the interrupt flag status by reading it from the ESP peripheral register
    const uint32_t intr_flags = dmx_uart_get_interrupt_status(dmx_num);
    if (intr_flags == 0)
    {
      // If there are no valid interrupt flags set then leave the ISR
      break;
    }

    // DMX Receive (DMX_INTR_RX_ALL) ####################################################
    // This section of code is entered when there is either:
    // 1) RS-485 Received Data
    // 2) RS-485 BREAK
    // 3) RS-485 Error (i.e. Overflow, Framing, etc)
    if (intr_flags & DMX_INTR_RX_ALL) 
    {
      // For debugging purposes - set GPIO15 LOW when handling the UART Receive ISR bits
      // gpio_set_level(GPIO_NUM_15, 0);

      // If there is enough space remaining in the drivers dmx.data[] buffer then read data into the DMX buffer
      int dmx_head;
      taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      dmx_head = driver->dmx.head;
      taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      if ( (dmx_head >= 0) && (dmx_head < DMX_PACKET_SIZE_MAX) ) 
      {
        // Determine how much size is available to read and store in the buffer
        int read_len = DMX_PACKET_SIZE_MAX - dmx_head;
        // Call the low level peripheral functions to read the data. Pass in read_len as the maximum amount of data that can be read from the peripheral buffer. 
        // If the number of bytes in the peripheral buffer is less than the read_len then read_len will be updated with how many bytes are actually available.
        dmx_uart_read_rxfifo(dmx_num, &driver->dmx.data[dmx_head], &read_len);
        // Increment both the head pointer of the buffer as well as the new dmxFrameLengthTracker to track how much data has been read in
        dmx_head += read_len;
        dmxFrameLengthTracker += read_len;
        taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
        driver->dmx.head = dmx_head;
        taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      } 
      else 
      {
        // There was not enough space in the buffer so reset
        if (dmx_head > 0) 
        {
          // Record the number of slots received for error reporting
          dmx_head += dmx_uart_get_rxfifo_len(dmx_num);
          taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
          driver->dmx.head = dmx_head;
          taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
        }
        dmx_uart_rxfifo_reset(dmx_num);
      }

      // Since right above here we've read the received data from the UART peripheral it is now OK to clear the Receive Data Flag
      dmx_uart_clear_interrupt(dmx_num, DMX_INTR_RX_DATA);

      // Handle UART error condition - the previous code handled the break before the error but again this could lead to race conditions when the 
      // cable insertion and removal caused framing errors.
      if (intr_flags & DMX_INTR_RX_ERR) 
      {
        // Since the receive error is being handled - clear the flag in the peripheral
        dmx_uart_clear_interrupt(dmx_num, DMX_INTR_RX_ERR);
        // Since a receive error has just happened - also clear break flag in the peripheral so the next frame is ignored 
        dmx_uart_clear_interrupt(dmx_num, DMX_INTR_RX_BREAK);

        // On error - reset all the driver variables
        taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));      
        driver->dmx.head = DMX_HEAD_WAITING_FOR_BREAK;
        driver->dmx.size = DMX_PACKET_SIZE_MAX; 
        driver->dmx.status = DMX_STATUS_IDLE;
        driver->dmx.progress = DMX_PROGRESS_STALE;
        // If a client was waiting on the data then raise the RTOS event to notify that the data is invalid (Improper Slot = Framing Error)
        if (driver->task_waiting) 
        {
          xTaskNotifyFromISR(driver->task_waiting, DMX_ERR_IMPROPER_SLOT, eSetValueWithOverwrite, &task_awoken);
        }  
        taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
        // Nothing else to do in this ISR on DMX error so break out and allow for re-entry during next ISR
        break;  
      } 
      else if (intr_flags & DMX_INTR_RX_BREAK) 
      {
        // Handle DMX break condition - when a break has been detected it means this is the start of a brand new DMX or RDM frame
        dmx_uart_clear_interrupt(dmx_num, DMX_INTR_RX_BREAK);
        
        // For debugging purposes - set GPIO13 LOW when handling the UART Break ISR bits
        // gpio_set_level(GPIO_NUM_13, 0);

        // Handle possible condition where expected packet size is too large
        if (driver->dmx.progress == DMX_PROGRESS_IN_DATA && dmx_head > 0) 
        {
          // For debugging purposes - set GPIO14 LOW when "recovering". This is when the UART Break is detected while the state machine was still processing valid DMX data
          // gpio_set_level(GPIO_NUM_14, 0);
          taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
          // Attempt to fix packet size
          driver->dmx.size = (dmx_head - 1);  
          if (driver->task_waiting) 
          {
            xTaskNotifyFromISR(driver->task_waiting, DMX_ERR_NOT_ENOUGH_SLOTS, eSetValueWithOverwrite, &task_awoken);
          }
          taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num)); 
          // For debugging purposes - set GPIO14 HIGH when "recovering" is done. This is when the UART Break is detected while the state machine was still processing valid DMX data
          // gpio_set_level(GPIO_NUM_14, 1); 
        }

        // Now handle the condition where the DMX Frame Length has changed - this typically happens when inserting and removing the cable
        // and data frames are not fully received. Once the data stream is consistent then the dmx.size will be equal to dmxFrameLengthTracker
        // and this logic will no longer be executed
        if( (dmxFrameLengthTracker != 0) && (dmxFrameLengthTracker != driver->dmx.size)) 
        {
          taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
          // Update dmx frame size based on how many bytes have been receieved between this BREAK and the last BREAK
          driver->dmx.size = (dmxFrameLengthTracker - 1);  
          taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
        }

        // Since this was a BREAK we can clear the dmxFrameLengthTracker back to 0 and begin counting again when the next BYTE is received
        dmxFrameLengthTracker = 0;

        // Reset the DMX buffer for the next packet
        taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
        driver->dmx.status = DMX_STATUS_RECEIVING;
        driver->dmx.progress = DMX_PROGRESS_IN_BREAK;
        driver->dmx.head = 0;
        taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));

        // For debugging purposes - set GPIO13 HIGH when done handling the UART Break ISR bits
        // gpio_set_level(GPIO_NUM_13, 1);  
        // Nothing else to do on DMX break so continue back up to the start of the ISR and see if there is anything else to process
        continue;
      } 
      else if ( (driver->dmx.progress == DMX_PROGRESS_IN_BREAK) || (driver->dmx.progress == DMX_PROGRESS_IN_MAB) ) 
      {
        taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
        // UART ISR cannot detect MAB so we go straight to DMX_PROGRESS_IN_DATA
        driver->dmx.progress = DMX_PROGRESS_IN_DATA;
        taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      }

      // Guard against notifying multiple times for the same packet
      if (driver->dmx.progress != DMX_PROGRESS_IN_DATA) 
      {
        continue;
      }

      // Process the data depending on the type of packet that was received
      dmx_err_t err = DMX_OK;
      int rdm_type;
      bool packet_is_complete;

      // Determine the type of the packet that was received
      const uint8_t sc = driver->dmx.data[0];  // DMX start-code.
      if (sc == RDM_SC) 
      {
        rdm_type = RDM_TYPE_IS_UNKNOWN;  // Determine actual type later
      } 
      else if (sc == RDM_PREAMBLE || sc == RDM_DELIMITER) 
      {
        rdm_type = RDM_TYPE_IS_DISCOVERY;
      } 
      else 
      {
        rdm_type = RDM_TYPE_IS_NOT_RDM;
        taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
        // Get the best resolution on the controller EOP timestamp
        driver->dmx.controller_eop_timestamp = now;
        taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      }

      // Set inter-slot timer for RDM response packets
      if ( (driver->is_controller) && (rdm_type != RDM_TYPE_IS_NOT_RDM) ) 
      {
        dmx_timer_set_counter(dmx_num, 0);
        dmx_timer_set_alarm(dmx_num, RDM_TIMING_RESPONDER_INTER_SLOT_MAX, false);
      }

      while (err == DMX_OK) 
      {
        if (rdm_type == RDM_TYPE_IS_DISCOVERY) 
        {
          // Parse an RDM discovery response packet
          if (dmx_head < 17) 
          {
            packet_is_complete = false;
            break;  // Haven't received the minimum packet size
          }

          // Get the delimiter index
          int delimiter_idx = 0;
          for (; delimiter_idx <= 7; ++delimiter_idx) 
          {
            const uint8_t slot_value = driver->dmx.data[delimiter_idx];
            if (slot_value != RDM_PREAMBLE) 
            {
              if (slot_value != RDM_DELIMITER) 
              {
                delimiter_idx = 9;  // Force invalid packet type
              }
              break;
            }
          }
          if (delimiter_idx > 8) 
          {
            rdm_type = RDM_TYPE_IS_NOT_RDM;
            continue;  // Packet is malformed - treat it as DMX
          }

          // Process RDM discovery response packet
          if (dmx_head < delimiter_idx + 17) 
          {
            packet_is_complete = false;
            break;  // Haven't received full RDM_PID_DISC_UNIQUE_BRANCH response
          } 
          else if (!rdm_read_header(dmx_num, NULL)) 
          {
            rdm_type = RDM_TYPE_IS_NOT_RDM;
            continue;  // Packet is malformed - treat it as DMX
          } 
          else 
          {
            driver->dmx.last_responder_pid = RDM_PID_DISC_UNIQUE_BRANCH;
            driver->dmx.responder_sent_last = true;
            packet_is_complete = true;
            break;
          }
        } 
        else if (rdm_type != RDM_TYPE_IS_NOT_RDM) 
        {
          // Parse a standard RDM packet
          uint8_t msg_len;
          if (dmx_head < sizeof(rdm_header_t) + 2) 
          {
            packet_is_complete = false;
            break;  // Haven't received full RDM header and checksum yet
          } 
          else if ( (driver->dmx.data[1] != RDM_SUB_SC) || (!rdm_cc_is_valid(driver->dmx.data[20])) || ((msg_len = driver->dmx.data[2]) < sizeof(rdm_header_t)) ) 
          {
            rdm_type = RDM_TYPE_IS_NOT_RDM;
            continue;  // Packet is malformed - treat it as DMX
          } 
          else if (dmx_head < msg_len + 2) 
          {
            packet_is_complete = false;
            break;  // Haven't received full RDM packet and checksum yet
          } 
          else if (!rdm_read_header(dmx_num, NULL)) 
          {
            rdm_type = RDM_TYPE_IS_NOT_RDM;
            continue;  // Packet is malformed - treat it as DMX
          } 
          else 
          {
            bool responder_sent_last;
            const rdm_cc_t cc = driver->dmx.data[20];
            const rdm_pid_t *pid = (rdm_pid_t *)&driver->dmx.data[21];
            const rdm_uid_t *uid_ptr = (rdm_uid_t *)&driver->dmx.data[3];
            const rdm_uid_t dest_uid = {.man_id = bswap16(uid_ptr->man_id),
                                        .dev_id = bswap32(uid_ptr->dev_id)};
            if (!rdm_cc_is_request(cc)) 
            {
              rdm_type = RDM_TYPE_IS_RESPONSE;
              responder_sent_last = true;
            } 
            else if (rdm_uid_is_broadcast(&dest_uid)) 
            {
              rdm_type = RDM_TYPE_IS_BROADCAST;
              responder_sent_last = false;
            } 
            else 
            {
              rdm_type = RDM_TYPE_IS_REQUEST;
              responder_sent_last = false;
            }

            if (!responder_sent_last) 
            {
              taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
              driver->dmx.controller_eop_timestamp = now;
              driver->dmx.last_controller_pid = bswap16(*pid);
              taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
            } 
            else 
            {
              taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
              driver->dmx.last_responder_pid = bswap16(*pid);
              taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
            }
            taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
            driver->dmx.responder_sent_last = responder_sent_last;
            taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
            packet_is_complete = true;
            break;
          }
        } 
        else 
        {
          // Parse a standard DMX packet
          // TODO: verify that a data collision hasn't happened
          taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
          driver->dmx.last_controller_pid = 0;
          driver->dmx.responder_sent_last = false;
          taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
          // This was the bug in the previous implmenetation of the code. We were getting here in the failure case since head is greater than size so SW thinks packet is complete.
          // This would occur immediately after the BREAK - the first slot that is processed will land us in here
          packet_is_complete = (dmx_head >= driver->dmx.size);
          break;
        }
      } // while (err == DMX_OK) 

      // When packet is not yet complete we jump back up to the top of the while(true)
      if (!packet_is_complete) 
      {
        continue;
      }

      dmx_timer_stop(dmx_num);

      // Set driver flags and notify task
      taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      driver->dmx.progress = DMX_PROGRESS_COMPLETE;
      driver->dmx.status = DMX_STATUS_IDLE;  // Could still be receiving data
      if (driver->task_waiting) 
      {
        xTaskNotifyFromISR(driver->task_waiting, err, eSetValueWithOverwrite, &task_awoken);
      }
      taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));

      // For debugging purposes - set GPIO15 HIGH when done handling the UART Receive ISR bits
      // gpio_set_level(GPIO_NUM_15, 1);
    }
    // DMX Transmit #####################################################
    else if (intr_flags & DMX_INTR_TX_DATA) 
    {
      // Write data to the UART and clear the interrupt
      int write_len = driver->dmx.size - driver->dmx.head;
      dmx_uart_write_txfifo(dmx_num, &driver->dmx.data[driver->dmx.head], &write_len);
      driver->dmx.head += write_len;
      dmx_uart_clear_interrupt(dmx_num, DMX_INTR_TX_DATA);

      // Allow FIFO to empty when done writing data
      if (driver->dmx.head == driver->dmx.size) 
      {
        dmx_uart_disable_interrupt(dmx_num, DMX_INTR_TX_DATA);
      }
    } 
    else if (intr_flags & DMX_INTR_TX_DONE) 
    {
      // Disable write interrupts and clear the interrupt
      dmx_uart_disable_interrupt(dmx_num, DMX_INTR_TX_ALL);
      dmx_uart_clear_interrupt(dmx_num, DMX_INTR_TX_DONE);

      // Record the EOP timestamp if this device is the DMX controller
      if (driver->is_controller) {
        taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
        driver->dmx.controller_eop_timestamp = now;
        taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      }

      // Update the DMX status and notify task
      taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      driver->dmx.progress = DMX_PROGRESS_COMPLETE;
      driver->dmx.status = DMX_STATUS_IDLE;
      if (driver->task_waiting) 
      {
        xTaskNotifyFromISR(driver->task_waiting, DMX_OK, eNoAction, &task_awoken);
      }
      taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));

      // Skip the rest of the ISR loop if an RDM response is not expected
      if (!driver->is_controller || driver->dmx.last_controller_pid == 0 ||
          (driver->dmx.last_request_was_broadcast && driver->dmx.last_controller_pid != RDM_PID_DISC_UNIQUE_BRANCH)) 
      {
        continue;
      }

      // Determine if a DMX break is expected in the response packet
      int progress;
      if (driver->dmx.last_controller_pid == RDM_PID_DISC_UNIQUE_BRANCH) 
      {
        progress = DMX_PROGRESS_IN_DATA;
        driver->dmx.head = 0;  // Not expecting a DMX break
      } 
      else 
      {
        progress = DMX_PROGRESS_STALE;
        driver->dmx.head = DMX_HEAD_WAITING_FOR_BREAK;
      }

      // Flip the DMX bus so the response may be read
      taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      dmx_uart_rxfifo_reset(dmx_num);
      dmx_uart_set_rts(dmx_num, 1);
      driver->dmx.progress = progress;
      taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
    }
  } // while (true) 

  if (task_awoken)
  {
    portYIELD_FROM_ISR();
  }

  // For debugging purposes - set GPIO14 HIGH when leaving the UART ISR
  // gpio_set_level(GPIO_NUM_14, 1);
}

bool dmx_uart_init(dmx_port_t dmx_num, void *isr_context, int isr_flags) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];

  ///@note BSR - fixed this compiler error
  // periph_module_enable(uart_periph_signal[dmx_num].module);
  switch( dmx_num )
  {
    case DMX_NUM_0:
      periph_module_enable(PERIPH_UART0_MODULE);
      break;
    
    case DMX_NUM_1:	
      periph_module_enable(PERIPH_UART1_MODULE);
      break;
    
    case DMX_NUM_2:
      periph_module_enable(PERIPH_UART2_MODULE);
      break;

    case DMX_NUM_MAX:
    default:
      break;
  }
  if (dmx_num != 0) {  // Default UART port for console
#if SOC_UART_REQUIRE_CORE_RESET
    // ESP32C3 workaround to prevent UART outputting garbage data
    uart_ll_set_reset_core(uart->dev, true);
    periph_module_reset(uart_periph_signal[dmx_num].module);
    uart_ll_set_reset_core(uart->dev, false);
#else
    ///@note BSR - fixed this compiler error
    // periph_module_reset(uart_periph_signal[dmx_num].module);
    switch( dmx_num )
    {
      case DMX_NUM_0:
        periph_module_reset(PERIPH_UART0_MODULE);
        break;
      
      case DMX_NUM_1:	
        periph_module_reset(PERIPH_UART1_MODULE);
        break;
      
      case DMX_NUM_2:
        periph_module_reset(PERIPH_UART2_MODULE);
        break;

      case DMX_NUM_MAX:
      default:
        break;
    } 
#endif
  }
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  uint32_t sclk_freq;
#if CONFIG_IDF_TARGET_ESP32C6
  // UART2 on C6 is a LP UART, with fixed GPIO pins for tx, rx, and rts
  if (dmx_num == 2) {
    LP_CLKRST.lpperi.lp_uart_clk_sel = 0;  // Use LP_UART_SCLK_LP_FAST
  } else {
    uart_ll_set_sclk(uart->dev, UART_SCLK_DEFAULT);
  }
  uart_get_sclk_freq(UART_SCLK_DEFAULT, &sclk_freq);
#else
  uart_ll_set_sclk(uart->dev, UART_SCLK_DEFAULT);
  uart_get_sclk_freq(UART_SCLK_DEFAULT, &sclk_freq);
#endif
  uart_ll_set_baudrate(uart->dev, DMX_BAUD_RATE, sclk_freq);
#else
  uart_ll_set_sclk(uart->dev, UART_SCLK_APB);
  uart_ll_set_baudrate(uart->dev, DMX_BAUD_RATE);
#endif
  uart_ll_set_mode(uart->dev, UART_MODE_UART);
  uart_ll_set_parity(uart->dev, UART_PARITY_DISABLE);
  uart_ll_set_data_bit_num(uart->dev, UART_DATA_8_BITS);
  uart_ll_set_stop_bits(uart->dev, UART_STOP_BITS_2);
  uart_ll_tx_break(uart->dev, 0);
  uart_ll_set_tx_idle_num(uart->dev, 0);
  uart_ll_set_hw_flow_ctrl(uart->dev, UART_HW_FLOWCTRL_DISABLE, 0);
  uart_ll_set_txfifo_empty_thr(uart->dev, DMX_UART_EMPTY_DEFAULT);
  uart_ll_set_rxfifo_full_thr(uart->dev, DMX_UART_FULL_DEFAULT);

  dmx_uart_rxfifo_reset(dmx_num);
  dmx_uart_txfifo_reset(dmx_num);
  dmx_uart_disable_interrupt(dmx_num, UART_LL_INTR_MASK);
  dmx_uart_clear_interrupt(dmx_num, UART_LL_INTR_MASK);

  esp_intr_alloc(uart_periph_signal[dmx_num].irq, isr_flags, dmx_uart_isr,
                 isr_context, &uart->isr_handle);

  return uart;
}

void dmx_uart_deinit(dmx_port_t dmx_num) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
  if (uart->num != 0) {  // Default UART port for console
    ///@note BSR - fixed this compiler error
    // periph_module_disable(uart_periph_signal[uart->num].module);
    switch( dmx_num )
    {
      case DMX_NUM_0:
        periph_module_disable(PERIPH_UART0_MODULE);
        break;
      
      case DMX_NUM_1:	
        periph_module_disable(PERIPH_UART1_MODULE);
        break;
      
      case DMX_NUM_2:
        periph_module_disable(PERIPH_UART2_MODULE);
        break;

      case DMX_NUM_MAX:
      default:
        break;
    } 
  }
}

// Temporary function just used for logging of the internal uart.c DMX Module parameters (since they cannot be logged from ISR where they are manipulated)
void dmx_log_debug(dmx_port_t dmx_num)
{
    dmx_driver_t *const driver = dmx_driver[dmx_num];

    int temp_head;      // The index of the slot being transmitted or received.
    int temp_size;      // The expected size of the incoming/outgoing packet.
    int temp_status;    // The status of the DMX port.
    int temp_progress;  // The progress of the current packet.

    taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
    temp_head = driver->dmx.head;
    temp_size = driver->dmx.size;
    temp_status = driver->dmx.status;
    temp_progress = driver->dmx.progress;
    taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
    if( temp_progress == DMX_PROGRESS_COMPLETE )
    {
      ESP_LOGD("ESP_DMX", "H(%d) S(%d) St(%d) P(%d)", temp_head, temp_size, temp_status, temp_progress);
      // Logging of all bytes to transmit
      // ESP_LOG_BUFFER_HEXDUMP("ESP_DMX", &(driver->dmx.data[0]), temp_head, ESP_LOG_DEBUG);
    }
}

bool dmx_uart_set_pin(dmx_port_t dmx_num, int tx, int rx, int rts) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
  esp_err_t err = uart_set_pin(uart->num, tx, rx, rts, -1);
  return (err == ESP_OK);
}

uint32_t dmx_uart_get_baud_rate(dmx_port_t dmx_num) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  uint32_t sclk_freq;
  uart_get_sclk_freq(UART_SCLK_DEFAULT, &sclk_freq);
  return uart_ll_get_baudrate(uart->dev, sclk_freq);
#else
  return uart_ll_get_baudrate(uart->dev);
#endif
}

void dmx_uart_set_baud_rate(dmx_port_t dmx_num, uint32_t baud_rate) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  uint32_t sclk_freq;
  uart_get_sclk_freq(UART_SCLK_DEFAULT, &sclk_freq);
  uart_ll_set_baudrate(uart->dev, baud_rate, sclk_freq);
#else
  uart_ll_set_baudrate(uart->dev, baud_rate);
#endif
}

void DMX_ISR_ATTR dmx_uart_invert_tx(dmx_port_t dmx_num, int invert) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
#if CONFIG_IDF_TARGET_ESP32C6
  uart->dev->conf0_sync.txd_inv = invert;
  uart_ll_update(uart->dev);
#else
  uart->dev->conf0.txd_inv = invert;
#endif
}

int dmx_uart_get_rts(dmx_port_t dmx_num) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
#if CONFIG_IDF_TARGET_ESP32C6
  return uart->dev->conf0_sync.sw_rts;
#else
  return uart->dev->conf0.sw_rts;
#endif
}

int DMX_ISR_ATTR dmx_uart_get_interrupt_status(dmx_port_t dmx_num) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
  return uart_ll_get_intsts_mask(uart->dev);
}

void DMX_ISR_ATTR dmx_uart_enable_interrupt(dmx_port_t dmx_num, int mask) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
  uart_ll_ena_intr_mask(uart->dev, mask);
}

void DMX_ISR_ATTR dmx_uart_disable_interrupt(dmx_port_t dmx_num, int mask) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
  uart_ll_disable_intr_mask(uart->dev, mask);
}

void DMX_ISR_ATTR dmx_uart_clear_interrupt(dmx_port_t dmx_num, int mask) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
  uart_ll_clr_intsts_mask(uart->dev, mask);
}

uint32_t DMX_ISR_ATTR dmx_uart_get_rxfifo_len(dmx_port_t dmx_num) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
  return uart_ll_get_rxfifo_len(uart->dev);
}

void DMX_ISR_ATTR dmx_uart_read_rxfifo(dmx_port_t dmx_num, uint8_t *buf,
                                       int *size) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
  const int rxfifo_len = uart_ll_get_rxfifo_len(uart->dev);
  if (*size > rxfifo_len) {
    *size = rxfifo_len;
  }
  uart_ll_read_rxfifo(uart->dev, buf, *size);
}

void DMX_ISR_ATTR dmx_uart_set_rts(dmx_port_t dmx_num, int set) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
  uart_ll_set_rts_active_level(uart->dev, set);
}

void DMX_ISR_ATTR dmx_uart_rxfifo_reset(dmx_port_t dmx_num) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
  uart_ll_rxfifo_rst(uart->dev);
}

uint32_t DMX_ISR_ATTR dmx_uart_get_txfifo_len(dmx_port_t dmx_num) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
  return uart_ll_get_txfifo_len(uart->dev);
}

void DMX_ISR_ATTR dmx_uart_write_txfifo(dmx_port_t dmx_num, const void *buf,
                                        int *size) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
  const int txfifo_len = uart_ll_get_txfifo_len(uart->dev);
  if (*size > txfifo_len) *size = txfifo_len;
  uart_ll_write_txfifo(uart->dev, (uint8_t *)buf, *size);
}

void DMX_ISR_ATTR dmx_uart_txfifo_reset(dmx_port_t dmx_num) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
  uart_ll_txfifo_rst(uart->dev);
}
