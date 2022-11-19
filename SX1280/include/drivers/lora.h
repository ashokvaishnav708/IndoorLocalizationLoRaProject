/*
 * Copyright (c) 2019 Manivannan Sadhasivam
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_LORA_H_
#define ZEPHYR_INCLUDE_DRIVERS_LORA_H_

/**
 * @file
 * @brief Public LoRa APIs
 */

#include <zephyr/types.h>
#include <device.h>

#ifdef __cplusplus
extern "C" {
#endif

//#define CONFIG_LORA_SX1280

#ifdef CONFIG_LORA_SX1280
/*!
 * \brief Represents the bandwidth values for LORA packet type
 */
enum lora_signal_bandwidth {
	BW_0200 = 0x34,
	BW_0400 = 0x26,
	BW_0800 = 0x18,
	BW_1600 = 0x0A,
};
#else
enum lora_signal_bandwidth {
	BW_125_KHZ = 0,
	BW_250_KHZ,
	BW_500_KHZ,
};
#endif

#ifdef CONFIG_LORA_SX1280
/*!
 * \brief Represents the possible spreading factor values in LORA packet types
 */
enum lora_datarate {
	SF_5 = 0x50,
	SF_6 = 0x60,
	SF_7 = 0x70,
	SF_8 = 0x80,
	SF_9 = 0x90,
	SF_10 = 0xA0,
	SF_11 = 0xB0,
	SF_12 = 0xC0,
};
#else
enum lora_datarate {
	SF_5 = 5,
	SF_6,
	SF_7,
	SF_8,
	SF_9,
	SF_10,
	SF_11,
	SF_12,
};
#endif

enum lora_coding_rate {
	CR_4_5 = 1,
	CR_4_6 = 2,
	CR_4_7 = 3,
	CR_4_8 = 4,
};

struct lora_modem_config {
	uint32_t frequency;
	enum lora_signal_bandwidth bandwidth;
	enum lora_datarate datarate;
	enum lora_coding_rate coding_rate;
	uint16_t preamble_len;
	int8_t tx_power;
	bool tx;
};

struct lora_ranging_params {
	bool status;
	uint8_t RSSIReg;
	int16_t RSSIVal;
	double distance; // in centimeters
};

/**
 * @typedef lora_api_config()
 * @brief Callback API for configuring the LoRa module
 *
 * @see lora_config() for argument descriptions.
 */
typedef int (*lora_api_config)(const struct device *dev, struct lora_modem_config *config);

/**
 * @typedef lora_api_send()
 * @brief Callback API for sending data over LoRa
 *
 * @see lora_send() for argument descriptions.
 */
typedef int (*lora_api_send)(const struct device *dev, uint8_t *data, uint32_t data_len);

/**
 * @typedef lora_api_send_async()
 * @brief Callback API for sending data asynchronously over LoRa
 *
 * @see lora_send_async() for argument descriptions.
 */
typedef int (*lora_api_send_async)(const struct device *dev, uint8_t *data, uint32_t data_len,
				   struct k_poll_signal *async);

/**
 * @typedef lora_api_recv()
 * @brief Callback API for receiving data over LoRa
 *
 * @see lora_recv() for argument descriptions.
 */
typedef int (*lora_api_recv)(const struct device *dev, uint8_t *data, uint8_t size,
			     k_timeout_t timeout, int16_t *rssi, int8_t *snr);

/**
 * @typedef lora_api_test_cw()
 * @brief Callback API for transmitting a continuous wave
 *
 * @see lora_test_cw() for argument descriptions.
 */
typedef int (*lora_api_test_cw)(const struct device *dev, uint32_t frequency, int8_t tx_power,
				uint16_t duration);

//
//
//
typedef bool (*lora_api_setup_ranging)(const struct device *dev, struct lora_modem_config *config,
				       uint32_t address, uint8_t role);

typedef struct lora_ranging_params (*lora_api_transmit_ranging)(const struct device *dev,
								struct lora_modem_config *config,
								uint32_t address);

typedef int (*lora_api_receive_ranging)(const struct device *dev, struct lora_modem_config *config,
					uint32_t address, k_timeout_t timeout);

//
//
//

struct lora_driver_api {
	lora_api_config config;
	lora_api_send send;
	lora_api_send_async send_async;
	lora_api_recv recv;
	lora_api_test_cw test_cw;
	//
	lora_api_setup_ranging setup_ranging;
	lora_api_transmit_ranging transmit_ranging;
	lora_api_receive_ranging receive_ranging;
	//
};

//
//

static inline bool lora_setup_ranging(const struct device *dev, struct lora_modem_config *config,
				      uint32_t address, uint8_t role)
{
	const struct lora_driver_api *api = (const struct lora_driver_api *)dev->api;
	return api->setup_ranging(dev, config, address, role);
}

static inline struct lora_ranging_params
lora_transmit_ranging(const struct device *dev, struct lora_modem_config *config, uint32_t address)
{
	const struct lora_driver_api *api = (const struct lora_driver_api *)dev->api;
	return api->transmit_ranging(dev, config, address);
}

static inline int lora_receive_ranging(const struct device *dev, struct lora_modem_config *config,
				       uint32_t address, k_timeout_t timeout)
{
	const struct lora_driver_api *api = (const struct lora_driver_api *)dev->api;
	return api->receive_ranging(dev, config, address, timeout);
}

//
//

/**
 * @brief Configure the LoRa modem
 *
 * @param dev     LoRa device
 * @param config  Data structure containing the intended configuration for the
		  modem
 * @return 0 on success, negative on error
 */
static inline int lora_config(const struct device *dev, struct lora_modem_config *config)
{
	const struct lora_driver_api *api = (const struct lora_driver_api *)dev->api;

	return api->config(dev, config);
}

/**
 * @brief Send data over LoRa
 *
 * @note This blocks until transmission is complete.
 *
 * @param dev       LoRa device
 * @param data      Data to be sent
 * @param data_len  Length of the data to be sent
 * @return 0 on success, negative on error
 */
static inline int lora_send(const struct device *dev, uint8_t *data, uint32_t data_len)
{
	const struct lora_driver_api *api = (const struct lora_driver_api *)dev->api;

	return api->send(dev, data, data_len);
}

/**
 * @brief Asynchronously send data over LoRa
 *
 * @note This returns immediately after starting transmission, and locks
 *       the LoRa modem until the transmission completes.
 *
 * @param dev       LoRa device
 * @param data      Data to be sent
 * @param data_len  Length of the data to be sent
 * @param async A pointer to a valid and ready to be signaled
 *        struct k_poll_signal. (Note: if NULL this function will not
 *        notify the end of the transmission).
 * @return 0 on success, negative on error
 */
static inline int lora_send_async(const struct device *dev, uint8_t *data, uint32_t data_len,
				  struct k_poll_signal *async)
{
	const struct lora_driver_api *api = (const struct lora_driver_api *)dev->api;

	return api->send_async(dev, data, data_len, async);
}

/**
 * @brief Receive data over LoRa
 *
 * @note This is a blocking call.
 *
 * @param dev       LoRa device
 * @param data      Buffer to hold received data
 * @param size      Size of the buffer to hold the received data. Max size
		    allowed is 255.
 * @param timeout   Duration to wait for a packet.
 * @param rssi      RSSI of received data
 * @param snr       SNR of received data
 * @return Length of the data received on success, negative on error
 */
static inline int lora_recv(const struct device *dev, uint8_t *data, uint8_t size,
			    k_timeout_t timeout, int16_t *rssi, int8_t *snr)
{
	const struct lora_driver_api *api = (const struct lora_driver_api *)dev->api;

	return api->recv(dev, data, size, timeout, rssi, snr);
}

/**
 * @brief Transmit an unmodulated continuous wave at a given frequency
 *
 * @note Only use this functionality in a test setup where the
 * transmission does not interfere with other devices.
 *
 * @param dev       LoRa device
 * @param frequency Output frequency (Hertz)
 * @param tx_power  TX power (dBm)
 * @param duration  Transmission duration in seconds.
 * @return 0 on success, negative on error
 */
static inline int lora_test_cw(const struct device *dev, uint32_t frequency, int8_t tx_power,
			       uint16_t duration)
{
	const struct lora_driver_api *api = (const struct lora_driver_api *)dev->api;

	if (api->test_cw == NULL) {
		return -ENOSYS;
	}

	return api->test_cw(dev, frequency, tx_power, duration);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_LORA_H_ */
