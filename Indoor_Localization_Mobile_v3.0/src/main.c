

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

#define ROLE_SENDER 0x01

#define MAX_DATA_LEN 249

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL

// Operations
#define RANGING_INIT 0x00
#define RANGING_DONE 0x01
#define RANGING_ACK 0x02
#define RANGIGN_MODE_MASTER 0x03
#define RECEIVE 0x05

#define ALL_DONE_PKT 0x06
#define START_RANGING 0x07
#define ANCHOR_PKT 0x10
#define RE_RANGING_PKT 0x11
#define CORNER_PKT 0x12
#define NONE 0xFF
//

LOG_MODULE_REGISTER(Indoor_Localization_Mobile);

const uint16_t TxtimeoutmS = 5000;
const uint16_t RxtimeoutmS = 0xFFFF;
const uint32_t device_address = 01;

struct __attribute__((__packed__)) Coordinates
{
    bool flag; //  Validated Coordinates if TRUE else not validated
    float x;
    float y;
};

/*
************ Payload Format ************
DEVICE_ID | OPERATION | DATA_POINTER
*/

struct __attribute__((__packed__)) Payload
{
    uint32_t host_id;
    uint8_t operation;
    struct Coordinates coords;
};

struct Anchor
{
    uint32_t host_id;
    struct Coordinates coords;
    float distance;
    int16_t RSSI;
    struct Anchor *next;
};

struct Coordinates bottom_left_corner = {
    .flag = false,
    .x = -1,
    .y = -1,
};
struct Coordinates top_right_corner = {
    .flag = false,
    .x = -1,
    .y = -1,
};

/* ********* Queue Operations ********** */

struct Anchor *front = NULL;
struct Anchor *rear = NULL;
int anchor_count = 0;

bool already_existing(uint32_t host_id)
{
    struct Anchor *temp_anchor;
    temp_anchor = front;
    do
    {
        if (temp_anchor->host_id == host_id)
        {
            return true;
        }
        temp_anchor = temp_anchor->next;
    } while (temp_anchor != NULL);
    return false;
}

bool add_anchor(uint32_t host_id, struct Coordinates coords)
{
    struct Anchor *n_anchor = malloc(sizeof(struct Anchor));
    n_anchor->coords = coords;
    n_anchor->host_id = host_id;
    n_anchor->distance = -1;
    // n_anchor->re_distance = -1;
    n_anchor->RSSI = 0;
    // n_anchor->re_RSSI = 0;
    n_anchor->next = NULL;

    if (rear == NULL)
    {
        front = n_anchor;
        rear = n_anchor;
    }
    else
    {
        if (already_existing(n_anchor->host_id))
            return false;
        rear->next = n_anchor;
        rear = rear->next;
    }
    anchor_count++;
    return true;
}

void remove_anchor(struct Anchor *prev_anchor, struct Anchor *anchor_ptr)
{
    if (prev_anchor == NULL)
    {
        front = anchor_ptr->next;
    }
    else
    {
        if (anchor_ptr->next == NULL)
        {
            if (prev_anchor == NULL)
                rear = NULL;
            else
                rear = prev_anchor;
        }
        else
        {
            prev_anchor->next = anchor_ptr->next;
        }
    }
    free(anchor_ptr);
    anchor_count--;
}

void remove_all_anchors()
{
    struct Anchor *temp_anchor;
    if (front != NULL)
    {
        do
        {
            temp_anchor = front;
            front = front->next;
            free(temp_anchor);
        } while (front != NULL);
    }

    rear = NULL;
    anchor_count = 0;
}

void show_anchors()
{
    struct Anchor *temp_anchor;
    if (front != NULL)
    {
        temp_anchor = front;
        do
        {
            LOG_INF("Anchor ID: %x RSSI: %d  Distance: %d",
                    temp_anchor->host_id, temp_anchor->RSSI, temp_anchor->distance);
            // LOG_INF("Re: Anchor ID: %x RSSI: %d  Distance: %d",
            //        temp_anchor->host_id, temp_anchor->re_RSSI, temp_anchor->re_distance);
            temp_anchor = temp_anchor->next;
        } while (temp_anchor != NULL);
    }
    else
    {
        LOG_INF("No Anchors Yet.");
    }
}

/*****************    Intersection Points Queue Operation     *****************/

struct IPs
{
    struct Coordinates coords;
    struct IPs *next;
};

#define IPs_LIST1 0x00
#define IPs_LIST2 0x01
#define POLYGON_IPs 0x02

struct IPs *ips_list1_front = NULL;
struct IPs *ips_list1_rear = NULL;

struct IPs *ips_list2_front = NULL;
struct IPs *ips_list2_rear = NULL;

struct IPs *ips_polygon_front = NULL;
struct IPs *ips_polygon_rear = NULL;

bool equate_coordinates(struct Coordinates coord1, struct Coordinates coord2)
{
    if ((coord1.x == coord2.x) && (coord1.y == coord2.y))
        return true;
    else
        return false;
}

bool already_existing_intersection(struct Coordinates coords, uint8_t IPs_Type)
{
    struct IPs *temp_ip = NULL;

    if (IPs_Type == IPs_LIST1)
    {
        temp_ip = ips_list1_front;
    }
    else if (IPs_Type == IPs_LIST2)
    {
        temp_ip = ips_list2_front;
    }

    do
    {
        if ((equate_coordinates(coords, temp_ip->coords)))
        {
            return true;
        }
        temp_ip = temp_ip->next;
    } while (temp_ip != NULL);
    return false;
}

void add_intersection(struct Coordinates coords, uint8_t IPs_Type)
{

    struct IPs *n_ip = malloc(sizeof(struct IPs));
    n_ip->coords = coords;
    n_ip->next = NULL;

    if (IPs_Type == IPs_LIST1)
    {
        if (ips_list1_rear == NULL)
        {
            ips_list1_front = n_ip;
            ips_list1_rear = n_ip;
        }
        else
        {
            if (!(already_existing_intersection(coords, IPs_Type)))
            {
                ips_list1_rear->next = n_ip;
                ips_list1_rear = ips_list1_rear->next;
            }
        }
    }
    else if (IPs_Type == IPs_LIST2)
    {
        if (ips_list2_rear == NULL)
        {
            ips_list2_front = n_ip;
            ips_list2_rear = n_ip;
        }
        else
        {
            if (!already_existing_intersection(coords, IPs_Type))
            {
                ips_list2_rear->next = n_ip;
                ips_list2_rear = ips_list2_rear->next;
            }
        }
    }
}

int get_intersection_count(uint8_t IPs_Type)
{
    int count = 0;
    struct IPs *temp_ip;

    if (IPs_Type == IPs_LIST1)
    {
        if (ips_list1_front != NULL)
        {
            temp_ip = ips_list1_front;
            do
            {
                // LOG_INF("IP: [%d, %d]", temp_ip->coords.x, temp_ip->coords.y);
                count++;
                temp_ip = temp_ip->next;
            } while (temp_ip != NULL);
        }
        else
            LOG_INF("Intersection Points Queue Empty.");
    }
    else if (IPs_Type == IPs_LIST2)
    {
        if (ips_list2_front != NULL)
        {
            temp_ip = ips_list2_front;
            do
            {
                // LOG_INF("IP: [%d, %d]", temp_ip->coords.x, temp_ip->coords.y);
                count++;
                temp_ip = temp_ip->next;
            } while (temp_ip != NULL);
        }
        else
            LOG_INF("Intersection Points Queue Empty.");
    }
    return count;
}

void remove_all_intersections(uint8_t IPs_Type)
{
    struct IPs *temp_ip;

    if (IPs_Type == IPs_LIST1)
    {
        if (ips_list1_front != NULL)
        {
            do
            {
                temp_ip = ips_list1_front;
                ips_list1_front = ips_list1_front->next;
                free(temp_ip);
            } while (ips_list1_front != NULL);
        }
        ips_list1_rear = NULL;
    }
    else if (IPs_Type == IPs_LIST2)
    {
        if (ips_list2_front != NULL)
        {
            do
            {
                temp_ip = ips_list2_front;
                ips_list2_front = ips_list2_front->next;
                free(temp_ip);
            } while (ips_list2_front != NULL);
        }
        ips_list2_rear = NULL;
    }
}

/*
bool equate_intersections(struct Coordinates coords, uint8_t IPs_Type)
{

    struct IPs *temp_ip = NULL;

    if (IPs_Type == IPs_LIST1)
    {
        temp_ip = ips_list1_front;
    }
    else if (IPs_Type == IPs_LIST2)
    {
        temp_ip = ips_list2_front;
    }
    do
    {
        if (equate_coordinates(temp_ip->coords, coords))
        {
            return true;
        }
        temp_ip = temp_ip->next;
    } while (temp_ip != NULL);
    return false;
}
*/

bool is_inside_building(struct Coordinates coords)
{
    float x = coords.x;
    float y = coords.y;
    if (x > bottom_left_corner.x && x < top_right_corner.x && y > bottom_left_corner.y && y < top_right_corner.y)
    {
        return true;
    }
    else
        return false;
}

/*
Removes duplicates from IPs_1 list to IPs_2 list
*/
/*
int remove_duplicate_intersections()
{
    LOG_INF("In Removing Duplicate Intersection Points.");
    int count = 0;
    struct IPs *temp_ip;
    if (ips_list1_front != NULL)
    {
        do
        {
            temp_ip = ips_list1_front;
            if (ips_list2_rear == NULL)
            {
                add_intersection(temp_ip->coords, IPs_LIST2);
            }
            else
            {
                if (!(equate_intersections(temp_ip->coords, IPs_LIST2)))
                {
                    if (is_inside_building(temp_ip->coords))
                    {
                        add_intersection(temp_ip->coords, IPs_LIST2);
                        count++;
                    }
                }
            }
            ips_list1_front = ips_list1_front->next;
            free(temp_ip);
        } while (ips_list1_front != NULL);
    }
    ips_list1_rear = NULL;
    return count;
}
*/

/**********************************************************************************/
/************************** Geometrical Mathematics Code **************************/
/**********************************************************************************/

float square(float x)
{
    return x * x;
}

/*
bool coordinates_set_intersection(struct Coordinates *coord0, struct Coordinates *coord, struct Coordinates *coord_prime)
{
    if (equate_coordinates(coord0, coord))
        return true;
    else if (equate_coordinates(coord0, coord_prime))
        return true;
    else
        return false;
}
*/

struct Coordinates get_centroid(uint8_t IPs_Type)
{
    struct Coordinates coords;
    struct IPs *temp_ip = NULL;
    int count = 0;
    // LOG_INF("In Calculating centroid.");

    if (IPs_Type == IPs_LIST1)
        temp_ip = ips_list1_front;
    else if (IPs_Type == IPs_LIST2)
        temp_ip = ips_list2_front;

    do
    {
        coords.x += temp_ip->coords.x;
        coords.y += temp_ip->coords.y;

        count++;
        temp_ip = temp_ip->next;
    } while (temp_ip != NULL);

    coords.x = (coords.x / count);
    coords.y = (coords.y / count);
    coords.flag = true;
    return coords;
}

void circles_intersection(struct Anchor *anchor1, struct Anchor *anchor2, struct Coordinates *coord, struct Coordinates *coord_prime)
{

    //  struct Coordinates coord, coord_prime;
    float x2, y2, dx, dy;

    float dist, a, h;

    dx = anchor2->coords.x - anchor1->coords.x;
    dy = anchor2->coords.y - anchor1->coords.y;

    // LOG_INF("In Calculating circle intersection.");

    dist = hypot(dx, dy);

    // Check if two circles are not same || circles do not meet || one circle is inside another one
    if ((dist == 0.0 && anchor1->distance == anchor2->distance) || (dist > (anchor1->distance + anchor2->distance)) || (dist < fabs(anchor1->distance - anchor2->distance)))
    {
        // LOG_INF("Intersection not possible.");
        coord->flag = false;
        coord_prime->flag = false;
    }
    else
    {
        a = (square(anchor1->distance) - square(anchor2->distance) + square(dist)) / (2.0 * dist);
        h = sqrt(square(anchor1->distance) - square(a));

        x2 = anchor1->coords.x + (dx * a / dist);
        y2 = anchor1->coords.y + (dy * a / dist);

        coord->x = ceil(x2 + (dy * h / dist));
        coord_prime->x = ceil(x2 - (dy * h / dist));

        coord->y = ceil(y2 - (dx * h / dist));
        coord_prime->y = ceil(y2 + (dx * h / dist));

        coord->flag = true;
        coord_prime->flag = true;

        // LOG_INF("Intersection : (%d, %d) (%d, %d)", coord->x, coord->y, coord_prime->x, coord_prime->y);
    }
}

bool is_inside(struct Anchor *anchor, struct Coordinates coords)
{

    float circle_x = anchor->coords.x;
    float circle_y = anchor->coords.y;

    float x = coords.x;
    float y = coords.y;
    // if (!(is_inside_building(coords)))
    //     return false;
    if (square(x - circle_x) + square(y - circle_y) <= square((anchor->distance) + 5))
        return true;
    else
        return false;
}

int is_inside_circles(struct Coordinates coords)
{

    int count = 0;
    struct Anchor *temp_anchor;
    temp_anchor = front;
    do
    {
        if (temp_anchor->distance > 0)
        {
            if (is_inside(temp_anchor, coords))
            {
                count++;
            }
        }
        temp_anchor = temp_anchor->next;
    } while (temp_anchor != NULL);
    return count;
}

int get_polygon(uint8_t from_Type, uint8_t to_Type, int error, int active_anchors)
{
    int count = 0;
    int circle_count = 0;
    struct IPs *temp_ip = NULL;

    if (from_Type == IPs_LIST1)
        temp_ip = ips_list1_front;
    else if (from_Type == IPs_LIST2)
        temp_ip = ips_list2_front;

    do
    {
        circle_count = is_inside_circles(temp_ip->coords);
        // LOG_INF("In %d circles", circle_count);
        if (circle_count >= (active_anchors - error))
        {
            add_intersection(temp_ip->coords, to_Type);
            count++;
        }
        temp_ip = temp_ip->next;
    } while (temp_ip != NULL);
    return count;
}

struct Coordinates get_dev_location()
{
    struct Coordinates dev_coords = {
        .flag = false,
        .x = -1,
        .y = -1,
    };
    struct Coordinates coord, coord_prime;

    struct Coordinates *coord_ptr, *coord_prime_ptr;
    coord_ptr = &coord;
    coord_prime_ptr = &coord_prime;

    struct Anchor *anchor_ptr_i;
    struct Anchor *anchor_ptr_j;
    int ret;
    //  bool first_iter = true;
    //  dev_coord.flag = NULL;
    //  temp_coord.flag = NULL;

    int error_rate = 0;
    int active_anchors = 0;

    coord.flag = false;
    coord_prime.flag = false;

    anchor_ptr_i = front;
    do
    {
        if (anchor_ptr_i->distance > 0)
        {
            anchor_ptr_j = front;
            do
            {
                if (anchor_ptr_i != anchor_ptr_j)
                {
                    if (anchor_ptr_j->distance > 0)
                    {
                        circles_intersection(anchor_ptr_i, anchor_ptr_j, coord_ptr, coord_prime_ptr);
                        // LOG_INF("Coordinates Flags %d %d", coord.flag, coord_prime.flag);
                        if (coord.flag && coord_prime.flag)
                        {
                            // LOG_INF("In Adding Intersction.");
                            add_intersection(coord, IPs_LIST1);
                            add_intersection(coord_prime, IPs_LIST1);
                            // get_intersection_count(ips_front, ips_rear);
                        }
                    }
                }

                anchor_ptr_j = anchor_ptr_j->next;
            } while (anchor_ptr_j != NULL);

            active_anchors++;
        }

        anchor_ptr_i = anchor_ptr_i->next;
    } while (anchor_ptr_i != NULL);

    // LOG_INF("Total Intersections: %d", get_intersection_count(IPs_LIST1));
// remove_all_intersections(IPs_LIST1);

/*
// Below function dumps the non-duplicate data in new IPs queue and frees the previous IPs list
ret = remove_duplicate_intersections();
LOG_INF("Filtered Intersections: %d", ret);
remove_all_intersections(IPs_LIST1);
*/
POLYGON_CALC:
    ret = get_polygon(IPs_LIST1, IPs_LIST2, error_rate, active_anchors);
    // LOG_INF("Polygon Points: %d", ret);

    if (ret > 2)
    {
        dev_coords = get_centroid(IPs_LIST2);
        remove_all_intersections(IPs_LIST2);
    }
    /*
    else if (ret > 1)
    {
        dev_coords = get_centroid(IPs_LIST2);
        remove_all_intersections(IPs_LIST2);
    }
    */

    else
    {
        if (error_rate < 1)
        {
            error_rate = 1;
            goto POLYGON_CALC;
        }
        remove_all_intersections(IPs_LIST2);
    }

    remove_all_intersections(IPs_LIST1);
    return dev_coords;
}

// main function
void main(void)
{
    // Getting Device Id
    uint8_t hwid[4];
    ssize_t length;
    length = hwinfo_get_device_id(hwid, sizeof(hwid));
    uint32_t host_id = (hwid[0] << 24) | (hwid[1] << 16) | (hwid[2] << 8) | hwid[3];

    struct Coordinates dev_coords = {.flag = false, .x = -1, .y = -1};

    const struct device *lora_dev = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
    struct lora_modem_config config;

    // Payload declaration
    struct Payload payload;
    uint8_t *payload_ptr;
    payload_ptr = &payload;

    struct Anchor *anchor_ptr;
    // struct Anchor *prev_anchor = NULL;
    // struct Anchor *temp_anchor;
    struct lora_ranging_params ranging_result;

    // General Variables
    int16_t rssi;
    int8_t snr;
    int ret, len;
    bool ranging_done = false;
    uint8_t operation = RECEIVE;

    int sample_count = 5;
    float sum = 0;
    float avg_fact = 0;
    int samples = 0;
    float ratio = 2570 / 1992;
    float avg_dist = 0;
    bool anchor_pkt_possible = false;

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

    // BEGIN:
    while (1)
    {

        // Setup LoRa Device
        if (ranging_done)
        {
            ret = lora_config(lora_dev, &config);
            if (ret < 0)
            {
                LOG_ERR("LoRa config failed");
                return;
            }
            ranging_done = false;
        }

        switch (operation)
        {
        case RECEIVE:
            LOG_INF("RECEIVE MODE.");
            len = lora_recv(lora_dev, payload_ptr, MAX_DATA_LEN, K_MSEC(1000),
                            &rssi, &snr);
            if (len < 0)
            {
                if (len == -(EAGAIN))
                {
                    remove_all_anchors();
                    operation = RANGING_INIT;
                }
                else
                    operation = RECEIVE;
            }
            else
            {
                if (payload.operation == RANGING_INIT)
                    operation = RECEIVE;
                else
                    operation = payload.operation;
            }
            break;
        case RANGING_INIT:
            // remove_all_anchors();
            LOG_INF("RANGING INIT PKT BROADCASTED.");
            payload.operation = RANGING_INIT;
            payload.host_id = host_id;
            payload.coords = dev_coords;

            k_sleep(K_MSEC(20));
            ret = lora_send(lora_dev, payload_ptr, sizeof(payload));
            k_sleep(K_MSEC(30));

            operation = RECEIVE;
            break;
        case CORNER_PKT:
            if (payload.host_id == host_id)
            {
                LOG_INF("Corner Packet Received");
                if (payload.coords.flag == false)
                {
                    bottom_left_corner.flag = true;
                    bottom_left_corner.x = payload.coords.x;
                    bottom_left_corner.y = payload.coords.y;
                }
                else
                {
                    top_right_corner.flag = true;
                    top_right_corner.x = payload.coords.x;
                    top_right_corner.y = payload.coords.y;

                    anchor_pkt_possible = true;
                }
            }

            operation = RECEIVE;
            break;

        case ANCHOR_PKT:
            if (anchor_pkt_possible)
            {
                if (!add_anchor(payload.host_id, payload.coords))
                {
                    LOG_INF("Received Already Existing Anchor.");
                }
            }
            operation = RECEIVE;
            break;
        case ALL_DONE_PKT:
            if (payload.host_id != host_id)
            {
                operation = RECEIVE;
                break;
            }
            LOG_INF("ALL ANCHORS RECEIVED.");
            // show_anchors();
            if (anchor_count > 2)
            {
                lora_setup_ranging(lora_dev, &config, host_id, ROLE_SENDER);
                operation = START_RANGING;
            }
            else
                operation = RECEIVE;

            break;

        case START_RANGING:
            // k_sleep(K_MSEC(10));

            anchor_ptr = front;
            do
            {
                // k_sleep(K_MSEC(30));
                samples = 0;
                sum = 0;
                avg_fact = 0;

                while (samples < sample_count)
                {
                    ranging_result = lora_transmit_ranging(lora_dev, &config, (anchor_ptr->host_id));
                    if (ranging_result.status != false && ranging_result.distance > 0)
                    {
                        sum = sum + ranging_result.distance;
                        avg_fact++;
                    }

                    samples++;
                }
                if (sum > 0.0)
                {
                    avg_dist = sum / avg_fact;
                    anchor_ptr->distance = ceil(avg_dist * ratio); // Distance in pixels via ratio multiplication.
                    anchor_ptr->RSSI = ranging_result.RSSIVal;
                }
                else
                {
                    anchor_ptr->distance = -1;
                    anchor_ptr->RSSI = 0;
                }

                anchor_ptr = anchor_ptr->next;

            } while (anchor_ptr != NULL);

            // show_anchors();
            //  prev_anchor = NULL;
            dev_coords = get_dev_location();
            // ranging_done = true;
            if (dev_coords.flag)
                LOG_INF("Device Location :(%d, %d).", dev_coords.x, dev_coords.y);

            /*
            if(anchor_count >= 3)
            {
                dev_coords = calc_dev_location();
                if (dev_coords.flag == false) LOG_ERR("Location Ambiguious");
                else LOG_INF("Device Location : (%d, %d)", dev_coords.x, dev_coords.y);
            }
            */
            operation = START_RANGING;
            break;

        default:
            operation = RECEIVE;
        }
    }
}