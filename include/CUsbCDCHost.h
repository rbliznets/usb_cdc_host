#pragma once

#include "sdkconfig.h"
#include "CBaseTask.h"
#include "esp_pm.h"
#include <atomic>
#include <memory>

#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"

/*
 * Параметры задачи
 */
#define USBHOSTTASK_NAME "usbhost"		   ///< Имя задачи для отладки.
#define USBHOSTTASK_STACKSIZE (4 * 1024) ///< Размер стека задачи (296).
#define USBHOSTTASK_LENGTH (5)		   ///< Длина приемной очереди задачи.

#define MSG_DEVICE_DISCONNECTED (100)
#define MSG_SEND_DATA (101)

/// Функция события приема данных.
/*!
 * \param[in] data данные.
 * \param[in] size размер данных.
 */
typedef void onUsbCDCHostDataRx(const uint8_t *data, size_t size);
typedef void onUsbCDCHostFailed(int error);
typedef void onUsbCDCHostConnected(bool connected);

struct SUsbCDCHostConfig
{
	onUsbCDCHostDataRx *onDataRx = nullptr; ///< Функция события приема данных
	onUsbCDCHostFailed *onFailed = nullptr;
	onUsbCDCHostConnected* onConnected = nullptr;

	uint8_t cpu = 1;   ///< Номер ядра процессора.
	uint8_t prior = 3; ///< Приоритет задачи.

	cdc_acm_line_coding_t line_coding = {
            .dwDTERate = 115200,
            .bCharFormat = 0,
            .bParityType = 0,
            .bDataBits = 8,
        };
};


class CUsbCDCHost : public CBaseTask
{
private:
	// Singleton instance pointer - points to the single instance of this class
	static CUsbCDCHost *theSingleInstance;

	std::atomic<bool> mConnected = false;
	std::atomic<uint16_t> mExit = 0;
protected:
#if CONFIG_PM_ENABLE
	// Power management lock handle to prevent CPU frequency reduction during USB operations
	esp_pm_lock_handle_t mPMLock;
#endif
    static bool handle_rx(const uint8_t *data, size_t data_len, void *arg);
    static void handle_event(const cdc_acm_host_dev_event_data_t *event, void *user_ctx);
    static void usb_lib_task(void *arg);

	const SUsbCDCHostConfig* mSets;
	/// Функция задачи.
    virtual void run() override;

    void connectDone(CdcAcmDevice* vcp);

    CUsbCDCHost(const SUsbCDCHostConfig* sets);
	virtual ~CUsbCDCHost();

public:
    static CUsbCDCHost *Instance(const SUsbCDCHostConfig* sets = nullptr)
	{
		if (theSingleInstance == nullptr)
			theSingleInstance = new CUsbCDCHost(sets);
		return theSingleInstance;
	};

	/// Free the singleton instance and release all resources.
	/// Should be called when USB functionality is no longer needed.
	static void free()
	{
		if (theSingleInstance != nullptr)
		{
			delete theSingleInstance;
			theSingleInstance = nullptr;
		}
	};

	static inline bool isRun() { return (theSingleInstance != nullptr); };

	inline bool isConnected() {return mConnected;};

	bool sendData(uint8_t *data, size_t size, uint32_t timeout = 10);

};
