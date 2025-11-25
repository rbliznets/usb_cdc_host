/**
 * @file
 * @brief Wrapper class for USB CDC host
 * @authors Bliznets R.A. (r.bliznets@gmail.com)
 * @version 0.0.0.1
 * @date 25.11.2025
 */

#pragma once

#include "sdkconfig.h"
#include "CBaseTask.h"
#include "esp_pm.h"
#include <atomic>
#include <memory>

#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"

/*
 * Task parameters
 */
#define USBHOSTTASK_NAME "usbhost"		 ///< Task name for debugging.
#define USBHOSTTASK_STACKSIZE (3 * 1024) ///< Task stack size (296).
#define USBHOSTTASK_LENGTH (5)			 ///< Task receive queue length.

#define MSG_DEVICE_DISCONNECTED (100)
#define MSG_SEND_DATA (101)

/**
 * @brief Data reception event callback function.
 * @param[in] data received data.
 * @param[in] size size of the data.
 */
typedef void onUsbCDCHostDataRx(const uint8_t *data, size_t size);
typedef void onUsbCDCHostFailed(int error);
typedef void onUsbCDCHostConnected(bool connected);

/**
 * @brief Configuration structure for USB CDC Host
 */
struct SUsbCDCHostConfig
{
	onUsbCDCHostDataRx *onDataRx = nullptr; ///< Data reception event callback function
	onUsbCDCHostFailed *onFailed = nullptr; ///< Callback for error events
	onUsbCDCHostConnected *onConnected = nullptr; ///< Callback for connection status changes

	uint8_t cpu = 1;   ///< CPU core number.
	uint8_t prior = 3; ///< Task priority.

	cdc_acm_line_coding_t line_coding = {
		.dwDTERate = 115200,  // Baud rate
		.bCharFormat = 0,     // Character format (0 = 1 stop bit)
		.bParityType = 0,     // Parity type (0 = none)
		.bDataBits = 8,       // Data bits (5, 6, 7, 8)
	};
};

/**
 * @brief USB CDC Host wrapper class
 * 
 * This class provides a singleton wrapper for USB CDC (Communication Device Class) host functionality,
 * allowing communication with USB serial devices like FTDI, CP210x, and CH34x chips.
 * It handles device connection, data transmission, and event callbacks in a separate task.
 */
class CUsbCDCHost : public CBaseTask
{
private:
	// Singleton instance pointer - points to the single instance of this class
	static CUsbCDCHost *theSingleInstance;

	std::atomic<bool> mConnected = false;  ///< Atomic flag indicating device connection status
	std::atomic<uint16_t> mExit = 0;      ///< Atomic exit status flag

protected:
#if CONFIG_PM_ENABLE
	// Power management lock handle to prevent CPU frequency reduction during USB operations
	esp_pm_lock_handle_t mPMLock;
#endif
	// Static callback functions for USB data reception and device events
	static bool handle_rx(const uint8_t *data, size_t data_len, void *arg);
	static void handle_event(const cdc_acm_host_dev_event_data_t *event, void *user_ctx);
	// Static task function for handling USB library events
	static void usb_lib_task(void *arg);

	const SUsbCDCHostConfig *mSets;  ///< Pointer to configuration settings
	/// Task function.
	virtual void run() override;

	// Helper function to handle successful connection
	void connectDone(CdcAcmDevice *vcp);

	/**
	 * @brief Private constructor to enforce singleton pattern
	 * @param[in] sets Configuration settings for the USB CDC host
	 */
	CUsbCDCHost(const SUsbCDCHostConfig *sets);
	// Virtual destructor
	virtual ~CUsbCDCHost();

public:
	/**
	 * @brief Get singleton instance, create if it doesn't exist
	 * @param[in] sets Optional configuration settings. If nullptr, default settings are used.
	 * @return Pointer to the singleton instance
	 */
	static CUsbCDCHost *Instance(const SUsbCDCHostConfig *sets = nullptr)
	{
		if (theSingleInstance == nullptr)
			theSingleInstance = new CUsbCDCHost(sets);
		return theSingleInstance;
	};

	/**
	 * @brief Free the singleton instance and release all resources.
	 * Should be called when USB functionality is no longer needed.
	 */
	static void free()
	{
		if (theSingleInstance != nullptr)
		{
			delete theSingleInstance;
			theSingleInstance = nullptr;
		}
	};

	/**
	 * @brief Check if singleton instance exists
	 * @return true if instance exists, false otherwise
	 */
	static inline bool isRun() { return (theSingleInstance != nullptr); };

	/**
	 * @brief Check if device is currently connected
	 * @return true if device is connected, false otherwise
	 */
	inline bool isConnected() { return mConnected; };

	/**
	 * @brief Send data to the connected USB device
	 * @param[in] data Pointer to data buffer to send
	 * @param[in] size Size of data to send in bytes
	 * @param[in] timeout Timeout for sending operation in ticks (default 10)
	 * @return true if data was sent successfully, false otherwise
	 */
	bool sendData(uint8_t *data, size_t size, uint32_t timeout = 10);
};