/**
 * @file
 * @brief Wrapper class for USB CDC host
 * @authors Bliznets R.A. (r.bliznets@gmail.com)
 * @version 0.0.0.1
 * @date 25.11.2025
 */

#include "CUsbCDCHost.h"
#include "CTrace.h"
#include "esp_log.h"
#include <cstring>
#include "esp_sleep.h"

#include "usb/vcp_ch34x.hpp"
#include "usb/vcp_cp210x.hpp"
#include "usb/vcp_ftdi.hpp"
#include "usb/vcp.hpp"
#include "usb/usb_host.h"

using namespace esp_usb;

CUsbCDCHost *CUsbCDCHost::theSingleInstance = nullptr;

#if CONFIG_LOG_DEFAULT_LEVEL >= 0
static const char *TAG = "CUsbCDCHost"; // Log tag for CUsbCDCHost module
#endif

#ifdef CONFIG_ESP_TASK_WDT
// Define maximum blocking time for task watchdog
#define TASK_MAX_BLOCK_TIME pdMS_TO_TICKS((CONFIG_ESP_TASK_WDT_TIMEOUT_S - 1) * 1000 + 500)
#else
#define TASK_MAX_BLOCK_TIME portMAX_DELAY // Infinite delay if watchdog is disabled
#endif

// Default USB CDC host configuration
static const SUsbCDCHostConfig defSets = {nullptr, nullptr, nullptr, 1, 3, {115200, 0, 0, 8}};

CUsbCDCHost::CUsbCDCHost(const SUsbCDCHostConfig *sets) : CBaseTask()
{
    if (sets == nullptr)
        mSets = &defSets; // Use default settings if no settings provided
    else
        mSets = sets;
#if CONFIG_PM_ENABLE
    // Create power management lock to maintain maximum APB frequency
    esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "rs485", &mPMLock);
#endif
    // Initialize base task with provided parameters
    CBaseTask::init(USBHOSTTASK_NAME, USBHOSTTASK_STACKSIZE, mSets->prior, USBHOSTTASK_LENGTH, mSets->cpu);
}

CUsbCDCHost::~CUsbCDCHost()
{
    sendCmd(MSG_END_TASK); // Send command to end the task
    do
    {
        vTaskDelay(1); // Delay 1 tick to allow task to process the command
    }
#if (INCLUDE_vTaskDelete == 1)
    while (mTaskHandle != nullptr); // Wait until task handle is nullified
#else
    while (mTaskQueue != nullptr); // Wait until task queue is nullified
#endif
#if CONFIG_PM_ENABLE
    esp_pm_lock_delete(mPMLock); // Delete power management lock
#endif
}

/**
 * @brief Data received callback
 *
 * @param[in] data     Pointer to received data
 * @param[in] data_len Length of received data in bytes
 * @param[in] arg      Argument we passed to the device open function
 * @return
 *   true:  We have processed the received data
 *   false: We expect more data
 */
bool CUsbCDCHost::handle_rx(const uint8_t *data, size_t data_len, void *arg)
{
    CUsbCDCHost *host = (CUsbCDCHost *)arg;
    if (host->mSets->onDataRx != nullptr)
    {
        // Call user-defined data reception callback
        host->mSets->onDataRx(data, data_len);
    }
    else
    {
        ESP_LOGI(TAG, "Data received");                            // Log received data info
        ESP_LOG_BUFFER_HEXDUMP(TAG, data, data_len, ESP_LOG_INFO); // Log data in hex format
    }
    return true; // Indicate data has been processed
}

/**
 * @brief Device event callback
 *
 * Apart from handling device disconnection it doesn't do anything useful
 *
 * @param[in] event    Device event type and data
 * @param[in] user_ctx Argument we passed to the device open function
 */
void CUsbCDCHost::handle_event(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    CUsbCDCHost *host = (CUsbCDCHost *)user_ctx;
    switch (event->type)
    {
    case CDC_ACM_HOST_ERROR:
        if (host->mSets->onFailed != nullptr)
        {
            // Call user-defined failure callback with error code
            host->mSets->onFailed(event->data.error);
        }
        else
            ESP_LOGE(TAG, "CDC-ACM error has occurred, err_no = %i", event->data.error);
        break;
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        host->mConnected = false; // Mark device as disconnected
        // Send device disconnected message to task queue
        host->sendCmd(MSG_DEVICE_DISCONNECTED, 0, 0, 10);
        break;
    case CDC_ACM_HOST_SERIAL_STATE:
        ESP_LOGI(TAG, "Serial state notif 0x%04X", event->data.serial_state.val);
        break;
    case CDC_ACM_HOST_NETWORK_CONNECTION:
    default:
        ESP_LOGW(TAG, "Unsupported CDC event: %i", event->type);
        break;
    }
}

/**
 * @brief USB Host library handling task
 *
 * @param arg CUsbCDCHost
 */
void CUsbCDCHost::usb_lib_task(void *arg)
{
    CUsbCDCHost *host = (CUsbCDCHost *)arg;
    while (true)
    {
        // Start handling system events
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)
        {
            // Free all USB devices when no clients are connected
            ESP_ERROR_CHECK(usb_host_device_free_all());
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE)
        {
            ESP_LOGI(TAG, "USB: All devices freed");
            if (host->mExit == 1) // Check if exit flag is set to 1
            {
                break; // Exit loop
            }
        }
        else if (host->mExit == 2) // Check if exit flag is set to 2
        {
            break; // Exit loop
        }
    }
    host->mExit = 3;      // Set exit flag to 3 to indicate task completion
    vTaskDelete(nullptr); // Delete this task
}

bool CUsbCDCHost::sendData(uint8_t *data, size_t size, uint32_t timeout)
{
    if (mConnected) // Check if device is connected
    {
        STaskMessage msg;
        // Allocate new message with data payload
        uint8_t *dt = allocNewMsg(&msg, MSG_SEND_DATA, size, true);
        std::memcpy(dt, data, size);        // Copy data to message buffer
        return sendMessage(&msg, 10, true); // Send message to task queue
    }
    else
        return false; // Return false if not connected
}

void CUsbCDCHost::run()
{
#ifndef CONFIG_FREERTOS_CHECK_STACKOVERFLOW_NONE
    UBaseType_t m1 = uxTaskGetStackHighWaterMark2(nullptr); // Get initial stack high water mark
#endif
    STaskMessage msg;

    // Install USB Host driver. Should only be called once in entire application
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,            // Don't skip PHY setup
        .intr_flags = ESP_INTR_FLAG_LOWMED, // Interrupt flags for USB
    };
    if (usb_host_install(&host_config) != ESP_OK)
    {
        ESP_LOGE(TAG, "Installing USB Host failed"); // Log installation failure
        return;
    }
    else
    {
        ESP_LOGI(TAG, "USB Host was installed"); // Log successful installation
    }
    // Create USB library handling task
    BaseType_t task_usb_lib = xTaskCreate(usb_lib_task, "usb_lib", 4096, this, 10, nullptr);

    if (cdc_acm_host_install(nullptr) != ESP_OK)
    {
        ESP_LOGE(TAG, "Installing CDC-ACM driver failed"); // Log CDC-ACM driver installation failure
        usb_host_uninstall();                              // Uninstall USB host if CDC-ACM installation fails
        return;
    }
    else
    {
        ESP_LOGI(TAG, "Installing CDC-ACM driver"); // Log CDC-ACM driver installation
        // Register VCP drivers to VCP service
        VCP::register_driver<FT23x>();  // Register FTDI driver
        VCP::register_driver<CP210x>(); // Register CP210x driver
        VCP::register_driver<CH34x>();  // Register CH34x driver
    }
    // Configuration for CDC-ACM device
    const cdc_acm_host_device_config_t dev_config = {
        .connection_timeout_ms = 1000, // Connection timeout in milliseconds
        .out_buffer_size = 512,        // Output buffer size
        .in_buffer_size = 512,         // Input buffer size
        .event_cb = handle_event,      // Event callback function
        .data_cb = handle_rx,          // Data callback function
        .user_arg = this,              // User argument passed to callbacks
    };
    // Attempt to open VCP device
    auto vcp = std::unique_ptr<CdcAcmDevice>(VCP::open(&dev_config));
    TickType_t delay = TASK_MAX_BLOCK_TIME; // Initial delay for message queue
    if (vcp == nullptr)
    {
        delay = 0; // No delay if device failed to open
    }
    else
    {
        connectDone(vcp.get()); // Handle successful connection
    }

    for (;;) // Main loop
    {
        if (getMessage(&msg, delay)) // Get message from queue with specified delay
        {
            switch (msg.msgID) // Process message based on ID
            {
            case MSG_SEND_DATA:
                if (mConnected && (vcp != nullptr)) // Check if connected and device is valid
                {
                    // Attempt to send data using blocking transmission
                    if (vcp->tx_blocking((uint8_t *)msg.msgBody, msg.shortParam, 100) != ESP_OK)
                    {
                        if (mSets->onFailed != nullptr)
                        {
                            mSets->onFailed(1); // Call failure callback with error code 1
                        }
                        ESP_LOGW(TAG, "tx_blocking failed"); // Log transmission failure
                    }
                }
                else if (mSets->onFailed != nullptr)
                {
                    mSets->onFailed(0);                    // Call failure callback with error code 0
                    ESP_LOGW(TAG, "MSG_SEND_DATA failed"); // Log send data failure
                }
                vPortFree(msg.msgBody); // Free message body memory
                break;
            case MSG_DEVICE_DISCONNECTED:
                if (vcp != nullptr) // Check if device pointer is valid
                {
                    vcp->close();  // Close the device
                    vcp = nullptr; // Nullify device pointer
                    if (mSets->onConnected != nullptr)
                    {
                        mSets->onConnected(false); // Call connection callback with disconnected state
                    }
                    else
                    {
                        ESP_LOGI(TAG, "Device suddenly disconnected"); // Log unexpected disconnection
                    }
                    delay = 0; // No delay for next message processing
                }
                break;
            case MSG_END_TASK:
                mConnected = false; // Mark as disconnected
                if (vcp != nullptr) // If device is still open
                {
                    mExit = 1;    // Set exit flag to 1
                    vcp->close(); // Close the device
                    if (mSets->onConnected != nullptr)
                    {
                        mSets->onConnected(false); // Call connection callback with disconnected state
                    }
                }
                else
                {
                    mExit = 2;                  // Set exit flag to 2
                    usb_host_device_free_all(); // Free all USB devices
                }
                goto endUsbHostTask; // Jump to cleanup section
            default:
                TRACE_WARNING("CUsbCDCHost:unknown message", msg.msgID); // Log unknown message
                break;
            }
        }
        if (vcp == nullptr) // Try to reconnect if device is not open
        {
            vcp = std::unique_ptr<CdcAcmDevice>(VCP::open(&dev_config)); // Attempt to open device
            if (vcp != nullptr)                                          // If successful
            {
                delay = TASK_MAX_BLOCK_TIME; // Reset delay
                connectDone(vcp.get());      // Handle successful connection
            }
        }

#ifndef CONFIG_FREERTOS_CHECK_STACKOVERFLOW_NONE
        UBaseType_t m2 = uxTaskGetStackHighWaterMark2(nullptr); // Get current stack high water mark
        if (m2 != m1)                                           // If stack usage has changed
        {
            m1 = m2;                        // Update previous mark
            TDEC("free usbhost stack", m2); // Log stack usage
        }
#endif
    }
endUsbHostTask:                              // Cleanup section
    esp_err_t er = cdc_acm_host_uninstall(); // Uninstall CDC-ACM host driver
    int count = 10;                          // Counter for waiting
    // Wait for USB library task to exit or timeout
    while ((mExit != 3) && (count > 0))
    {
        vTaskDelay(10); // Delay 10 ticks
        count--;        // Decrement counter
    }
    er |= usb_host_uninstall();            // Uninstall USB host driver
    ESP_LOGI(TAG, "usb host exit %d", er); // Log exit status
    while (getMessage(&msg, 0))
    {
        switch (msg.msgID) // Process message based on ID
        {
        case MSG_SEND_DATA:
            vPortFree(msg.msgBody); // Free message body memory
            break;
        default:
            break;
        }
    }
}

void CUsbCDCHost::connectDone(CdcAcmDevice *vcp)
{
    // Set line coding (baud rate, parity, etc.) for the device
    if (vcp->line_coding_set((cdc_acm_line_coding_t *)&(mSets->line_coding)) != ESP_OK)
    {
        if (mSets->onFailed != nullptr)
        {
            mSets->onFailed(2); // Call failure callback with error code 2
        }
        ESP_LOGW(TAG, "line_coding_set failed"); // Log line coding failure
    }

    if (mSets->onConnected != nullptr)
    {
        mSets->onConnected(true); // Call connection callback with connected state
    }
    else
    {
        ESP_LOGI(TAG, "Device connected"); // Log successful connection
    }
    mConnected = true; // Mark as connected
}
