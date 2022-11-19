#include <device.h>
#include <drivers/lora.h>
#include <errno.h>
#include <sys/util.h>
#include <zephyr.h>
#include <math.h>
#include <logging/log.h>
#include <drivers/hwinfo.h>
#include <drivers/gpio.h>
#include <drivers/hwinfo.h>

#define DEFAULT_RADIO_NODE DT_ALIAS(lora0)
BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_RADIO_NODE, okay),
             "No default LoRa radio specified in DT");

#define ROLE_SENDER 0x01
#define ROLE_RECEIVER 0x00

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <logging/log.h>

LOG_MODULE_REGISTER(sx1280_ranging);

void main(void)
{
    uint8_t hwid[4];
    ssize_t length;
    length = hwinfo_get_device_id(hwid, sizeof(hwid));
    uint32_t host_id = (hwid[0] << 24) | (hwid[1] << 16) | (hwid[2] << 8) | hwid[3];

    const struct device *lora_dev = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
    struct lora_modem_config config;

    if (!device_is_ready(lora_dev))
    {
        LOG_ERR("%s Device not ready.", lora_dev->name);
        return;
    }

    config.frequency = 2445000000;
    config.bandwidth = BW_1600;
    config.datarate = SF_9;
    config.preamble_len = 12;
    config.coding_rate = CR_4_5;
    config.tx_power = 10;
    config.tx = false;

    lora_setup_ranging(lora_dev, &config, host_id, ROLE_RECEIVER);

    while (1)
    {
        lora_receive_ranging(lora_dev, &config, host_id, K_FOREVER);
    }
}
