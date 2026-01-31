
#include "pico/btstack_flash_bank.h"

#include "btclient.h"
#include "btstack_tlv.h"
#include "btstack.h"

#define MAX_ATTRIB_VALUE_SIZE 512
#define INQUIRY_INTERVAL 5

typedef enum {
    BT_IDLE,
    BT_SCANNING,
    BT_CONNECTED,
    BT_TIMEOUT_THEN_RECONNECT,
    BT_TIMEOUT_THEN_SCAN
} bt_state_t;

bt_state_t bt_state;
bd_addr_t remote_addr;
uint16_t hid_host_cid = 0;
bool hid_host_descriptor_available = false;
hid_protocol_mode_t hid_host_report_mode = HID_PROTOCOL_MODE_REPORT;

btstack_packet_callback_registration_t hci_event_callback_registration;

// SDP
uint8_t hid_descriptor_storage[MAX_ATTRIB_VALUE_SIZE];

// TAG to store remove device address and type in TLV
#define TLV_TAG_HOGD ((((uint32_t) 'H') << 24) | (((uint32_t) 'O') << 16) | (((uint32_t) 'G') << 8) | ((uint32_t) 'D'))

// used to store remove device in TLV
const btstack_tlv_t* btstack_tlv_singleton_impl = NULL;
void* btstack_tlv_singleton_context;

// used for connection timeout and reconnect timer
btstack_timer_source_t connection_timer;

blink_state_t blink_state = BLINK_SLOW;

const hid_state_t default_state = {
    .buttons = 0,
    .buttons_toggled = 0,
    .lx = 0x80,
    .ly = 0x80,
    .rx = 0x80,
    .ry = 0x80,
    .l2 = 0x00,
    .r2 = 0x00,
    .hat = 0x8
};

hid_state_t latest_state;

void hid_disconnect()
{
    hid_host_cid = 0;
    hid_host_descriptor_available = false;
    memcpy(&latest_state, &default_state, sizeof(hid_state_t));
}

void btclient_get_hid_state(hid_state_t* dest)
{
    memcpy(dest, &latest_state, sizeof(hid_state_t));
    latest_state.buttons_toggled = 0;
}

void hid_host_handle_interrupt_report(const uint8_t* packet, uint16_t packet_len)
{
    if (packet_len < 1) return;
    if (*packet != 0xa1) return;

    packet++;
    packet_len--;

    btstack_hid_parser_t parser;
    btstack_hid_parser_init(&parser,
        hid_descriptor_storage_get_descriptor_data(hid_host_cid),
        hid_descriptor_storage_get_descriptor_len(hid_host_cid),
        HID_REPORT_TYPE_INPUT, packet, packet_len);

    uint16_t usage_page;
    uint16_t usage;
    int32_t  value;

    while (btstack_hid_parser_has_more(&parser)) {
        btstack_hid_parser_get_field(&parser, &usage_page, &usage, &value);
        switch (usage_page) {
        case 0x001:             // Generic desktop
            switch (usage) {
            case 0x30:          // X axis
                latest_state.lx = value;
                break;
            case 0x31:          // Y axis
                latest_state.ly = value;
                break;
            case 0x32:          // Rx axis
                latest_state.rx = value;
                break;
            case 0x33:          // left trigger
                latest_state.l2 = value;
                break;
            case 0x34:          // right trigger
                latest_state.r2 = value;
                break;
            case 0x35:          // Ry axis
                latest_state.ry = value;
                break;
            case 0x39:          // hat switch
                // hat values:
                // 0 - up
                // 1 - up-right
                // 2 - right
                // 3 - down-right
                // 4 - down
                // 5 - down-left
                // 6 - left
                // 7 - up-left
                // anything else - release
                latest_state.hat = value;
                break;
            default:
                break;
            }
            break;
        case 0x0009:            // Buttons
            if (usage == 0) {
                // no button pressed
                //latest_state.buttons = 0;
            } else {
                if (value) {
                    if (!(latest_state.buttons & (1 << (usage - 1)))) {
                        latest_state.buttons_toggled |= (1 << (usage - 1));
                    }
                    latest_state.buttons |= (1 << (usage - 1));
                } else {
                    if (latest_state.buttons & (1 << (usage - 1))) {
                        latest_state.buttons_toggled |= (1 << (usage - 1));
                    }
                    latest_state.buttons &= ~(1 << (usage - 1));
                }
            }
            break;
        }
    }

}

// Check if device is a HID gamepad
const uint32_t BT_MAJ_CLASS_MASK = 0x1f00;
const uint32_t BT_MIN_CLASS_MASK = 0x00fc;
const uint32_t BT_MAJ_CLASS_PERIPERAL = 0x0500;
const uint32_t BT_MIN_CLASS_GAMEPAD = 0x0008;
bool inq_result_dev_is_gamepad(const uint8_t* packet)
{
    uint32_t dev_class = gap_event_inquiry_result_get_class_of_device(packet);
    if ((dev_class & BT_MAJ_CLASS_MASK) == BT_MAJ_CLASS_PERIPERAL) {
        if (dev_class & BT_MIN_CLASS_GAMEPAD) {
            return true;
        }
    }
    return false;
}

void client_connect();

// Start scanning
void client_scan()
{
    bt_state = BT_SCANNING;
    blink_state = BLINK_SLOW;
    hid_disconnect();
    gap_inquiry_start(INQUIRY_INTERVAL);
}

void client_start_scan()
{
    if (btstack_tlv_singleton_impl) {
        uint len = btstack_tlv_singleton_impl->get_tag(btstack_tlv_singleton_context, TLV_TAG_HOGD, (uint8_t*)&remote_addr, sizeof(remote_addr));
        if (len == sizeof(remote_addr)) {
            client_connect();
            return;
        }
    }
    client_scan();
}

// Handle connection timeout
void client_connection_timeout(btstack_timer_source_t* ts)
{
    gap_inquiry_stop();
    client_scan();
}

// connect to remote device, but set a timer
void client_connect()
{
    // set timer
    blink_state = BLINK_MED;
    btstack_run_loop_set_timer(&connection_timer, 20000);
    btstack_run_loop_set_timer_handler(&connection_timer, client_connection_timeout);
    btstack_run_loop_add_timer(&connection_timer);
    bt_state = BT_CONNECTED;
    uint8_t status = hid_host_connect(remote_addr, hid_host_report_mode, &hid_host_cid);
}

// Handle timer event to trigger reconnect
void client_reconnect_timeout(btstack_timer_source_t* ts)
{
    switch (bt_state) {
    case BT_TIMEOUT_THEN_RECONNECT:
        client_connect();
        break;
    case BT_TIMEOUT_THEN_SCAN:
        client_start_scan();
        break;
    default:
        break;
    }
}

// Handle packets
void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size)
{
    uint8_t event;
    bd_addr_t event_addr;
    uint8_t status;

    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }

    event = hci_event_packet_get_type(packet);
    switch (event) {
    case BTSTACK_EVENT_STATE:
        // On boot, start scanning
        if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
            if (bt_state == BT_IDLE) {
                client_start_scan();
            }
        }
        break;
    case GAP_EVENT_INQUIRY_RESULT:
        if (bt_state != BT_SCANNING) break;
        if (!inq_result_dev_is_gamepad(packet)) break;
        // store address and type
        gap_event_inquiry_result_get_bd_addr(packet, remote_addr);
        // connect
        client_connect();
        break;
    case GAP_EVENT_INQUIRY_COMPLETE:
        switch (bt_state) {
        case BT_SCANNING:
            client_scan();
            break;
        case BT_CONNECTED:
            break;
        case BT_TIMEOUT_THEN_RECONNECT:
            break;
        case BT_TIMEOUT_THEN_SCAN:
            break;
        case BT_IDLE:
            break;
        default:
            break;
        }
        break;
    case HCI_EVENT_PIN_CODE_REQUEST:
        // if a pin code request is needed, respond with 0000
        hci_event_pin_code_request_get_bd_addr(packet, event_addr);
        gap_pin_code_response(event_addr, "0000");
        break;
    case HCI_EVENT_USER_CONFIRMATION_REQUEST:
        // A user needs to input a value being submitted...can't do this for this application
        break;
    case HCI_EVENT_DISCONNECTION_COMPLETE:
        //if (bt_state != BT_CONNECTED) break;
        switch (bt_state) {
        case BT_CONNECTED:
            bt_state = BT_TIMEOUT_THEN_RECONNECT;
            break;
        default:
            bt_state = BT_TIMEOUT_THEN_SCAN;
            break;
        }
        // set timer
        btstack_run_loop_set_timer(&connection_timer, 100);
        btstack_run_loop_set_timer_handler(&connection_timer, &client_reconnect_timeout);
        btstack_run_loop_add_timer(&connection_timer);
        break;
    case HCI_EVENT_HID_META:
        if (bt_state != BT_CONNECTED) break;
        switch (hci_event_hid_meta_get_subevent_code(packet)) {
        case HID_SUBEVENT_INCOMING_CONNECTION:
            btstack_run_loop_remove_timer(&connection_timer);
            // TODO: Check if we want this connection or not - what type of device???
            hid_host_accept_connection(hid_subevent_incoming_connection_get_hid_cid(packet), hid_host_report_mode);
            //hid_host_decline_connection(hid_subevent_incoming_connection_get_hid_cid(packet));
            break;
        case HID_SUBEVENT_CONNECTION_OPENED:
            // The status field of this event indicates if the control and interrupt
            // connections were opened successfully.
            btstack_run_loop_remove_timer(&connection_timer);
            status = hid_subevent_connection_opened_get_status(packet);
            if (status != ERROR_CODE_SUCCESS) {
                if (status == L2CAP_CONNECTION_RESPONSE_RESULT_REFUSED_SECURITY && !hid_host_descriptor_available) {
                    //blink_state = BLINK_MED;
                    uint32_t irq_status = save_and_disable_interrupts();
                    flash_range_erase(PICO_FLASH_BANK_STORAGE_OFFSET, PICO_FLASH_BANK_TOTAL_SIZE);
                    restore_interrupts(irq_status);
                }
                client_start_scan();
                return;
            }
            bt_state = BT_CONNECTED;
            hid_host_descriptor_available = false;
            hid_host_cid = hid_subevent_connection_opened_get_hid_cid(packet);
            break;
        case HID_SUBEVENT_DESCRIPTOR_AVAILABLE:
            // This event will follows HID_SUBEVENT_CONNECTION_OPENED event. 
            // For incoming connections, i.e. HID Device initiating the connection,
            // the HID_SUBEVENT_DESCRIPTOR_AVAILABLE is delayed, and some HID  
            // reports may be received via HID_SUBEVENT_REPORT event. It is up to 
            // the application if these reports should be buffered or ignored until 
            // the HID descriptor is available.
            status = hid_subevent_descriptor_available_get_status(packet);
            if (status == ERROR_CODE_SUCCESS) {
                hid_host_descriptor_available = true;
                blink_state = BLINK_FAST;
                if (btstack_tlv_singleton_impl) {
                    btstack_tlv_singleton_impl->store_tag(btstack_tlv_singleton_context, TLV_TAG_HOGD, (const uint8_t*)&remote_addr, sizeof(remote_addr));
                }
            }
            break;
        case HID_SUBEVENT_REPORT:
            // Handle input report.
            if (hid_host_descriptor_available) {
                hid_host_handle_interrupt_report(hid_subevent_report_get_report(packet), hid_subevent_report_get_report_len(packet));
            }
            break;
        case HID_SUBEVENT_SET_PROTOCOL_RESPONSE:
            // For incoming connections, the library will set the protocol mode of the
            // HID Device as requested in the call to hid_host_accept_connection. The event 
            // reports the result. For connections initiated by calling hid_host_connect, 
            // this event will occur only if the established report mode is boot mode.
            break;
        case HID_SUBEVENT_CONNECTION_CLOSED:
            // The connection was closed.
            client_start_scan();
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

int btclient_setup()
{
    if (cyw43_arch_init()) {
        return -1;
    }

    l2cap_init();

    // Initialize hid host
    hid_host_init(hid_descriptor_storage, sizeof(hid_descriptor_storage));
    hid_host_register_packet_handler(packet_handler);

    // Allow sniff mode requests by HID device and support role switch
    gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_SNIFF_MODE | LM_LINK_POLICY_ENABLE_ROLE_SWITCH);

    // try to become master on incoming connections
    hci_set_master_slave_policy(HCI_ROLE_MASTER);

    // register for HCI events
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // make discoverable to allow HID device to initiate connection
    gap_discoverable_control(1);

    bt_state = BT_IDLE;
    btstack_tlv_get_instance(&btstack_tlv_singleton_impl, &btstack_tlv_singleton_context);
    hid_disconnect();

    hci_power_control(HCI_POWER_ON);

    return 0;
}

