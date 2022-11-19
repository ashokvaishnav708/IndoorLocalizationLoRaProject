#include <device.h>
#include <devicetree.h>
#include <drivers/lora.h>
#include <errno.h>
#include <sys/util.h>
#include <zephyr.h>
#include <logging/log.h>
#include <drivers/hwinfo.h>

#define DEFAULT_RADIO_NODE DT_ALIAS(lora0)
BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_RADIO_NODE, okay),
	     "No default LoRa radio specified in DT");


#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL

LOG_MODULE_REGISTER(Hardware_Id_Collection);

void main(void)
{

    const struct device *lora_dev = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
    struct lora_modem_config config;
    int ret;
    uint8_t hwid[4];
    
     if (!device_is_ready(lora_dev)) {
		LOG_ERR("%s Device not ready", lora_dev->name);
		return;
	}

    config.frequency = 2445000000;
	config.bandwidth = BW_0800;
	config.datarate = SF_8;
	config.preamble_len = 12;
	config.coding_rate = CR_4_5;
	config.tx_power = 10;
    config.tx = true;

    ret = lora_config(lora_dev, &config);
	if (ret < 0) {
		LOG_ERR("LoRa config failed");
		return;
	}

    while(1)
    {
        ssize_t length;
        length = hwinfo_get_device_id(hwid, sizeof(hwid));
        LOG_INF("Hardware Id : %x %x %x %x", 
            hwid[0], hwid[1], hwid[2], hwid[3]);
        k_sleep(K_MSEC(1000));
    }

}
