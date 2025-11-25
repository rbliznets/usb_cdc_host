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
static const char *TAG = "CUsbCDCHost";
#endif

#ifdef CONFIG_ESP_TASK_WDT
#define TASK_MAX_BLOCK_TIME pdMS_TO_TICKS((CONFIG_ESP_TASK_WDT_TIMEOUT_S - 1) * 1000 + 500)
#else
#define TASK_MAX_BLOCK_TIME portMAX_DELAY
#endif

static const SUsbCDCHostConfig defSets = {nullptr, nullptr, nullptr, 1, 3, {115200, 0, 0, 8}};

CUsbCDCHost::CUsbCDCHost(const SUsbCDCHostConfig *sets) : CBaseTask()
{
    if (sets == nullptr)
        mSets = &defSets;
    else
        mSets = sets;
#if CONFIG_PM_ENABLE
    esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "rs485", &mPMLock);
#endif
    CBaseTask::init(USBHOSTTASK_NAME, USBHOSTTASK_STACKSIZE, mSets->prior, USBHOSTTASK_LENGTH, mSets->cpu);
}

CUsbCDCHost::~CUsbCDCHost()
{
    sendCmd(MSG_END_TASK);
    do
    {
        vTaskDelay(1);
    }
#if (INCLUDE_vTaskDelete == 1)
    while (mTaskHandle != nullptr);
#else
    while (mTaskQueue != nullptr);
#endif
#if CONFIG_PM_ENABLE
    esp_pm_lock_delete(mPMLock);
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
        host->mSets->onDataRx(data, data_len);
    }
    else
    {
        ESP_LOGI(TAG, "Data received");
        ESP_LOG_BUFFER_HEXDUMP(TAG, data, data_len, ESP_LOG_INFO);
    }
    return true;
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
            host->mSets->onFailed(event->data.error);
        }
        ESP_LOGE(TAG, "CDC-ACM error has occurred, err_no = %i", event->data.error);
        break;
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        host->mConnected = false;
        host->sendCmd(MSG_DEVICE_DISCONNECTED, 0, 0, 10);
        // cdc_acm_host_close(event->data.cdc_hdl);
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
 * @param arg Unused
 */
void CUsbCDCHost::usb_lib_task(void *arg)
{
    CUsbCDCHost *host = (CUsbCDCHost *)arg;
    while (true)
    {
        // Start handling system events
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        // ESP_LOGW(TAG, "events 0x%x", event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)
        {
            ESP_ERROR_CHECK(usb_host_device_free_all());
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE)
        {
            ESP_LOGI(TAG, "USB: All devices freed");
            if (host->mExit == 1)
            {
                break;
            }
        }
        else if(host->mExit == 2)
        {
            break;
        }
    }
    host->mExit = 3;
    ESP_LOGW(TAG, "USB: exit");
    vTaskDelete(nullptr);
}

bool CUsbCDCHost::sendData(uint8_t *data, size_t size, uint32_t timeout)
{
    if (mConnected)
    {
        STaskMessage msg;
        uint8_t *dt = allocNewMsg(&msg, MSG_SEND_DATA, size, true);
        std::memcpy(dt, data, size);
        return sendMessage(&msg, 10, true);
    }
    else
        return false;
}

void CUsbCDCHost::run()
{
#ifndef CONFIG_FREERTOS_CHECK_STACKOVERFLOW_NONE
    UBaseType_t m1 = uxTaskGetStackHighWaterMark2(nullptr);
#endif
    STaskMessage msg;

    // Install USB Host driver. Should only be called once in entire application
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LOWMED,
    };
    if (usb_host_install(&host_config) != ESP_OK)
    {
        ESP_LOGE(TAG, "Installing USB Host failed");
        return;
    }
    else
    {
        ESP_LOGI(TAG, "USB Host was installed");
    }
    BaseType_t task_usb_lib = xTaskCreate(usb_lib_task, "usb_lib", 4096, this, 10, nullptr);

    if (cdc_acm_host_install(nullptr) != ESP_OK)
    {
        ESP_LOGE(TAG, "Installing CDC-ACM driver failed");
        usb_host_uninstall();
        return;
    }
    else
    {
        ESP_LOGI(TAG, "Installing CDC-ACM driver");
        // Register VCP drivers to VCP service
        VCP::register_driver<FT23x>();
        VCP::register_driver<CP210x>();
        VCP::register_driver<CH34x>();
    }
    const cdc_acm_host_device_config_t dev_config = {
        .connection_timeout_ms = 1000, // 1 second, enough time to plug the device in or experiment with timeout
        .out_buffer_size = 512,
        .in_buffer_size = 512,
        .event_cb = handle_event,
        .data_cb = handle_rx,
        .user_arg = this,
    };
    auto vcp = std::unique_ptr<CdcAcmDevice>(VCP::open(&dev_config));
    TickType_t delay = TASK_MAX_BLOCK_TIME;
    if (vcp == nullptr)
    {
        delay = 0;
    }
    else
    {
        connectDone(vcp.get());
    }

    for (;;)
    {
        if (getMessage(&msg, delay))
        {
            switch (msg.msgID)
            {
            case MSG_SEND_DATA:
                if (mConnected && (vcp != nullptr))
                {
                    if (vcp->tx_blocking((uint8_t *)msg.msgBody, msg.shortParam, 100) != ESP_OK)
                    {
                        if (mSets->onFailed != nullptr)
                        {
                            mSets->onFailed(1);
                        }
                        ESP_LOGW(TAG, "tx_blocking failed");
                    }
                }
                else if (mSets->onFailed != nullptr)
                {
                    mSets->onFailed(0);
                    ESP_LOGW(TAG, "MSG_SEND_DATA failed");
                }
                vPortFree(msg.msgBody);
                break;
            case MSG_DEVICE_DISCONNECTED:
                if (vcp != nullptr)
                {
                    vcp->close();
                    vcp = nullptr;
                    if (mSets->onConnected != nullptr)
                    {
                        mSets->onConnected(false);
                    }
                    else
                    {
                        ESP_LOGI(TAG, "Device suddenly disconnected");
                    }
                    delay = 0;
                }
                break;
            case MSG_END_TASK:
                mConnected = false;
                if (vcp != nullptr)
                {
                    mExit = 1;
                    vcp->close();
                    if (mSets->onConnected != nullptr)
                    {
                        mSets->onConnected(false);
                    }
                }
                else
                {
                    mExit = 2;
                    usb_host_device_free_all();
                }
                goto endUsbHostTask;
            default:
                TRACE_WARNING("CUsbCDCHost:unknown message", msg.msgID);
                break;
            }
        }
        if (vcp == nullptr)
        {
            vcp = std::unique_ptr<CdcAcmDevice>(VCP::open(&dev_config));
            if (vcp != nullptr)
            {
                delay = TASK_MAX_BLOCK_TIME;
                connectDone(vcp.get());
            }
        }

#ifndef CONFIG_FREERTOS_CHECK_STACKOVERFLOW_NONE
        UBaseType_t m2 = uxTaskGetStackHighWaterMark2(nullptr);
        if (m2 != m1)
        {
            m1 = m2;
            TDEC("free usbhost stack", m2);
        }
#endif
    }
endUsbHostTask:
    esp_err_t er = cdc_acm_host_uninstall();
    int count = 10;
    while ((mExit != 3) && (count > 0))
    {
        vTaskDelay(10);
        count--;
    }
    er |= usb_host_uninstall();
    ESP_LOGI(TAG, "usb host exit %d", er);
}

void CUsbCDCHost::connectDone(CdcAcmDevice *vcp)
{
    if (vcp->line_coding_set((cdc_acm_line_coding_t *)&(mSets->line_coding)) != ESP_OK)
    {
        if (mSets->onFailed != nullptr)
        {
            mSets->onFailed(2);
        }
        ESP_LOGW(TAG, "line_coding_set failed");
    }

    if (mSets->onConnected != nullptr)
    {
        mSets->onConnected(true);
    }
    else
    {
        ESP_LOGI(TAG, "Device connected");
    }
    mConnected = true;
}
