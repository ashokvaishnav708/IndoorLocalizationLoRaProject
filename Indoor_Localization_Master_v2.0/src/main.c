#include <device.h>
#include <drivers/lora.h>
#include <errno.h>
#include <sys/util.h>
#include <zephyr.h>
#include <logging/log.h>
#include <drivers/hwinfo.h>
#include <stdlib.h>
#include <math.h>

#define DEFAULT_RADIO_NODE DT_ALIAS(lora0)
BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_RADIO_NODE, okay),
             "No default LoRa radio specified in DT");

#define MAX_DATA_LEN 255

// Anchors
/* Hardware Addresses of currently available nrf52840 Anchors
raspi03     48 d7 41 a7,
raspi06     1c 1b 65 6e,
raspi07     1d 5a 2a 62,
raspi10     e7 32 e3 a5,
raspi12     66 18 2c 94,
raspi16     50 9a 12 7c,
raspi17     d2 bf 5c 98
*/

#define MAX_ANCHORS 07

#define RASPI03 0x48d741a7
#define RASPI06 0x1c1b656e
#define RASPI07 0x1d5a2a62
#define RASPI10 0xe732e3a5
#define RASPI12 0x66182c94
#define RASPI16 0x509a127c
#define RASPI17 0xd2bf5c98
//#define MASTER      0x33d6f416
//

// Operations
#define RANGING_INIT 0x00
#define RANGING_DONE 0x01
#define RANGING_ACK 0x02
#define RANGIGN_MODE_MASTER 0x03
#define SEND_ACK_PKT 0x04
#define RECEIVE 0x05

#define START_RANGING 0x07
#define ALIVE 0x08
#define ALIVE_ACK 0x09
#define ALL_DONE_PKT 0x06
#define ANCHOR_PKT 0x10
#define CORNER_PKT 0x12
#define NONE 0xFF
//

LOG_MODULE_REGISTER(Indoor_Localization_Master);

struct __attribute__((__packed__)) Coordinates
{
    bool flag; //  Validated Coordinates if TRUE else not validated
    float x;
    float y;
};

/*
************ Payload Format ************
DEVICE_ID | OPERATION | DEVICE_COORDINATES
*/

struct __attribute__((__packed__)) Payload
{
    uint32_t host_id;
    uint8_t operation;
    struct Coordinates coords;
};

const uint32_t anchor_id[MAX_ANCHORS] = {RASPI03, RASPI06, RASPI07, RASPI10, RASPI12, RASPI16, RASPI17};

struct Anchor
{
    uint32_t host_id;
    struct Coordinates coords;
};

struct Coordinates bottom_left_corner = {
    .flag = false,
    .x = 0.0,
    .y = 0.0,
};
struct Coordinates top_right_corner = {
    .flag = true,
    .x = 2865,
    .y = 2865,
};

void get_anchor_coordinates(uint32_t host_id, struct Coordinates *coords)
{
    switch (host_id)
    {
    case RASPI03: // Location of RASPI02
        coords->flag = true;
        coords->x = 1530;
        coords->y = 135;
        break;
    case RASPI06:
        coords->flag = true;
        coords->x = 1530;
        coords->y = 135;
        break;
    case RASPI07: // Location of RASPI09
        coords->flag = true;
        coords->x = 2730;
        coords->y = 2005;
        break;
    case RASPI10:
        coords->flag = true;
        coords->x = 2730;
        coords->y = 2705;
        break;
    case RASPI12:
        coords->flag = true;
        coords->x = 2295;
        coords->y = 875;
        break;
    case RASPI16:
        coords->flag = true;
        coords->x = 135;
        coords->y = 135;
        break;
    case RASPI17:
        coords->flag = true;
        coords->x = 135;
        coords->y = 2415;
        break;
    default:
        coords->flag = false;
        coords->x = -1.0;
        coords->y = -1.0;
    }
}

/*
void get_host_coordinates(uint32_t host_id, struct Coordinates *coords)
{
    switch (host_id)
    {
    case RASPI03:
        coords->flag = true;
        coords->x = 14.44;
        coords->y = 19.92;
        break;
    case RASPI06:
        coords->flag = true;
        coords->x = 19.93;
        coords->y = 19.92;
        break;
    case RASPI07:
        coords->flag = true;
        coords->x = 19.93;
        coords->y = 12.71;
        break;
    case RASPI10:
        coords->flag = true;
        coords->x = 19.93;
        coords->y = 0.0;
        break;
    case RASPI12:
        coords->flag = true;
        coords->x = 16.0;
        coords->y = 13.0;
        break;
    case RASPI16:
        coords->flag = true;
        coords->x = 0.0;
        coords->y = 19.92;
        break;
    default:
        coords->flag = false;
        coords->x = -1.0;
        coords->y = -1.0;
    }
}
*/

void main(void)
{

    // Hardware information gain
    uint8_t hwid[4];
    _ssize_t length;
    length = hwinfo_get_device_id(hwid, sizeof(hwid));
    uint32_t host_id = (hwid[0] << 24) | (hwid[1] << 16) | (hwid[2] << 8) | hwid[3];
    struct Coordinates dev_coords;

    const struct device *lora_dev = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
    struct lora_modem_config config;

    int ret, len;
    int16_t rssi;
    int8_t snr;
    bool ranging_req_possible = true;
    int count = 0;
    uint32_t ranging_req_id = 0x0000;
    uint8_t operation = RECEIVE;

    // Payload declaration
    struct Payload payload;
    uint8_t *payload_ptr;
    payload_ptr = &payload;

    get_anchor_coordinates(host_id, &dev_coords);

    if (!device_is_ready(lora_dev))
    {
        LOG_ERR("%s Device not ready", lora_dev->name);
        return;
    }

    config.frequency = 2445000000;
    config.bandwidth = BW_1600;
    config.datarate = SF_9;
    config.preamble_len = 12;
    config.coding_rate = CR_4_5;
    config.tx_power = 10;
    config.tx = true;

    // Setup LoRa Device
    ret = lora_config(lora_dev, &config);
    if (ret < 0)
    {
        LOG_ERR("LoRa config failed");
        return;
    }

    while (1)
    {

        switch (operation)
        {
        case RECEIVE:
            LOG_INF("RECEIVE MODE.");
            len = lora_recv(lora_dev, payload_ptr, MAX_DATA_LEN, K_MSEC(5000),
                            &rssi, &snr);
            // LOG_INF("LEN : %d", len);
            if (len < 0)
            {
                operation = RECEIVE;
                break;
            }
            operation = payload.operation;
            break;

        case RANGING_INIT:
            LOG_INF("RANGING REQUEST RECEIVED.");
            if (ranging_req_possible)
            {
                k_sleep(K_MSEC(30));
                LOG_INF("RANGING POSSIBLE");
                ranging_req_id = payload.host_id;
                count = 0;
                operation = CORNER_PKT;
                ranging_req_possible = false;
                break;
            }
            operation = RECEIVE;
            break;

        case CORNER_PKT:
            LOG_INF("SENDING BUILDING CORNERS.");
            if (count < 1)
            {
                payload.host_id = ranging_req_id;
                payload.coords = bottom_left_corner;
                payload.operation = CORNER_PKT;

                k_sleep(K_MSEC(20));
                ret = lora_send(lora_dev, payload_ptr, sizeof(payload));
                k_sleep(K_MSEC(30));

                operation = CORNER_PKT;
                count++;
            }
            else
            {
                payload.host_id = ranging_req_id;
                payload.coords = top_right_corner;
                payload.operation = CORNER_PKT;

                k_sleep(K_MSEC(20));
                ret = lora_send(lora_dev, payload_ptr, sizeof(payload));
                k_sleep(K_MSEC(30));

                count = 0;
                operation = ANCHOR_PKT;
            }
            break;

        case ANCHOR_PKT:
            if (count == MAX_ANCHORS)
            {
                payload.operation = ALL_DONE_PKT;
                payload.host_id = ranging_req_id;
                payload.coords = dev_coords;

                k_sleep(K_MSEC(20));
                ret = lora_send(lora_dev, payload_ptr, sizeof(payload));
                k_sleep(K_MSEC(30));

                count = 0;

                ranging_req_possible = true;
                operation = RECEIVE;
                break;
            }

            payload.host_id = anchor_id[count];
            payload.operation = ANCHOR_PKT;
            get_anchor_coordinates(anchor_id[count], &payload.coords);

            k_sleep(K_MSEC(20));
            ret = lora_send(lora_dev, payload_ptr, sizeof(payload));
            k_sleep(K_MSEC(30));
            operation = ANCHOR_PKT;

            count++;
            break;
        /*
        case START_RANGING:
            payload.operation = START_RANGING;
            payload.host_id = ranging_req_id;
            payload.coords = dev_coords;

            k_sleep(K_MSEC(20));
            ret = lora_send(lora_dev, payload_ptr, sizeof(payload));
            k_sleep(K_MSEC(15));

            ranging_req_possible = false;
            reset_anchors_status();
            count = 0;
            operation = RECEIVE;
            break;
        */
        default:
            operation = RECEIVE;
        }
    }
}
