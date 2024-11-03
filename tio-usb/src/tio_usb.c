/**
 * @file tio_usb.c
 * @author Adam Page (adam.page@ambiq.com)
 * @brief Tileio USB API
 * @version 0.1
 * @date 2024-10-01
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "ns_ambiqsuite_harness.h"
#include "arm_math.h"

#include "ns_usb.h"
#include "vendor_device.h"
#include "usb_descriptors.h"

#include "ringbuffer.h"
#include "tio_usb.h"

#define TIO_USB_VENDOR_ID 0xCAFE
#define TIO_USB_PRODUCT_ID 0x0001
#define TIO_USB_START_IDX 0
#define TIO_USB_START_VAL 0x55
#define TIO_USB_SLOT_IDX 1
#define TIO_USB_TYPE_IDX 2
#define TIO_USB_DLEN_IDX 3
#define TIO_USB_DLEN_LEN 2
#define TIO_USB_DATA_IDX 5
#define TIO_USB_DATA_LEN 248
#define TIO_USB_CRC_IDX 253
#define TIO_USB_CRC_LEN 2
#define TIO_USB_STOP_IDX 255
#define TIO_USB_STOP_VAL 0xAA
#define TIO_USB_UIO_BUF_LEN (8)

#define TIO_USB_RX_BUFSIZE (4096)
#define TIO_USB_TX_BUFSIZE (4096)

static uint8_t tioDeviceId[6];
static char tioSerialId[13];


static uint8_t tioRxBuffer[TIO_USB_RX_BUFSIZE] = {0};
static uint8_t tioTxBuffer[TIO_USB_TX_BUFSIZE] = {0};

static uint8_t tioRxRingBufferData[TIO_USB_RX_BUFSIZE];
static rb_config_t tioRxRingBuffer = {
    .buffer = (void *)tioRxRingBufferData,
    .dlen = sizeof(uint8_t),
    .size = TIO_USB_RX_BUFSIZE,
    .head = 0,
    .tail = 0,
};

static usb_handle_t tioUsbHandle = NULL;
static ns_usb_config_t tioWebUsbConfig = {
    .api = &ns_usb_V1_0_0,
    .deviceType = NS_USB_VENDOR_DEVICE,
    .rx_buffer = tioRxBuffer,
    .rx_bufferLength = TIO_USB_RX_BUFSIZE,
    .tx_buffer = tioTxBuffer,
    .tx_bufferLength = TIO_USB_TX_BUFSIZE,
    .rx_cb = NULL,
    .tx_cb = NULL,
    .service_cb = NULL};


/**
 * @brief Get the device ID
 *
 * @param deviceId
 */
static void
tio_get_device_id(uint8_t *deviceId)
{
    am_hal_mcuctrl_device_t device;
    am_hal_mcuctrl_info_get(AM_HAL_MCUCTRL_INFO_DEVICEID, &device);
    // DeviceID formed by ChipID1 (32 bits) and ChipID0 (8-23 bits).
    memcpy(deviceId, &device.ui32ChipID1, sizeof(device.ui32ChipID1));
    // ui32ChipID0 bit 8-31 is test time during chip manufacturing
    deviceId[4] = (device.ui32ChipID0 >> 8) & 0xFF;
    deviceId[5] = (device.ui32ChipID0 >> 16) & 0xFF;
}

/**
 * @brief Convert device ID to serial ID
 *
 * @param deviceId
 * @param serialId
 * @param len
 */
static void
tio_device_id_to_serial_id(uint8_t *deviceId, char *serialId, size_t len)
{
    size_t i;
    size_t deviceIdLen = 6;

    for (i = 0; i < deviceIdLen; i++)
    {
        uint32_t lsb = deviceId[i] & 0x0F;
        uint32_t msb = (deviceId[i] >> 4) & 0x0F;
        serialId[i * 2 + 1] = lsb < 10 ? '0' + lsb : 'A' + lsb - 10;
        serialId[i * 2] = msb < 10 ? '0' + msb : 'A' + msb - 10;
    }
    // Fill the rest of the serialId with 0
    for (i = deviceIdLen * 2; i < len; i++)
    {
        serialId[i] = 0;
    }
}

/**
 * @brief Compute CRC16 on packet data
 *
 * @param data Packet data
 * @param length Packet length
 * @return uint16_t
 */
static uint16_t
tio_compute_crc16(const uint8_t *data, uint32_t length)
{
    uint32_t m_crcStart = 0xEF4A;
    uint32_t crc = m_crcStart;
    uint32_t j;
    uint32_t i;
    uint32_t byte;
    uint32_t temp;
    const uint32_t andValue = 0x8000U;
    const uint32_t xorValue = 0x1021U;

    for (j = 0; j < length; ++j)
    {
        byte = data[j];
        crc ^= byte << 8;
        for (i = 0; i < 8U; ++i)
        {
            temp = crc << 1;
            if (0UL != (crc & andValue))
            {
                temp ^= xorValue;
            }
            crc = temp;
        }
    }

    return (uint16_t)crc;
}

/**
 * @brief Validate the USB packet is correct
 *
 * @param packet USB packet
 * @param length Packet length
 * @return uint32_t
 */
static uint32_t
tio_usb_validate_packet(const uint8_t *packet, uint32_t length)
{
    // Decode the packet
    if (length != TIO_USB_PACKET_LEN)
    {
        ns_lp_printf("Invalid packet length\n");
        return 1;
    }
    uint8_t start = packet[TIO_USB_START_IDX];
    uint8_t slot = packet[TIO_USB_SLOT_IDX];
    uint8_t slotType = packet[TIO_USB_TYPE_IDX];

    uint16_t dlen = (packet[TIO_USB_DLEN_IDX + 1] << 8) | packet[TIO_USB_DLEN_IDX];
    uint16_t crc = (packet[TIO_USB_CRC_IDX + 1] << 8) | packet[TIO_USB_CRC_IDX];

    uint16_t stop = packet[TIO_USB_STOP_IDX];
    uint16_t computedCrc = tio_compute_crc16(packet + TIO_USB_DLEN_IDX, dlen + TIO_USB_DLEN_LEN);
    if (start != TIO_USB_START_VAL || stop != TIO_USB_STOP_VAL)
    {
        ns_lp_printf("Invalid start/stop byte %lu %lu\n", start, stop);
        return 1;
    }
    if (crc != computedCrc)
    {
        ns_lp_printf("Invalid CRC %x %x\n", crc, computedCrc);
        return 1;
    }
    if (slotType <= 1 && dlen > TIO_USB_DATA_LEN)
    {
        ns_lp_printf("Invalid data length for slot type %lu\n", slotType);
        return 1;
    }
    if (slotType == 2 && dlen != TIO_USB_UIO_BUF_LEN)
    {
        ns_lp_printf("Invalid data length for UIO\n");
        return 1;
    }
    return 0;
}

/**
 * @brief Callback for USB receive
 *
 * @param buffer Rx buffer
 * @param length Buffer length
 * @param args Tileio USB context
 */
static void
tio_usb_receive_handler(const uint8_t *buffer, uint32_t length, void *args)
{
    tio_usb_context_t *ctx = (tio_usb_context_t *)args;
    ringbuffer_push(&tioRxRingBuffer, (void *)buffer, length);
    uint8_t slotFrame[TIO_USB_PACKET_LEN];
    uint32_t skip = 0;
    while (ringbuffer_len(&tioRxRingBuffer) >= TIO_USB_PACKET_LEN)
    {
        ringbuffer_peek(&tioRxRingBuffer, slotFrame, TIO_USB_PACKET_LEN);
        skip = tio_usb_validate_packet(slotFrame, TIO_USB_PACKET_LEN);
        if (skip)
        {
            ringbuffer_seek(&tioRxRingBuffer, 1);
            continue;
        }
        // If valid, parse the slot frame and send it to the appropriate slot
        uint8_t slot = slotFrame[TIO_USB_SLOT_IDX];
        uint8_t slotType = slotFrame[TIO_USB_TYPE_IDX];
        uint16_t length = (slotFrame[TIO_USB_DLEN_IDX + 1] << 8) | slotFrame[TIO_USB_DLEN_IDX];
        // Slot signal or metrics
        if (slotType <= 1 && ctx->slot_update_cb != NULL)
        {
            ctx->slot_update_cb(slot, slotType, slotFrame + TIO_USB_DATA_IDX, length);
        }
        // Slot UIO
        else if (slotType == 2 && ctx->uio_update_cb != NULL)
        {
            ctx->uio_update_cb(slotFrame + TIO_USB_DATA_IDX, length);
        }
        ringbuffer_seek(&tioRxRingBuffer, TIO_USB_PACKET_LEN);
    }
}

/**
 * @brief Pack slot data into USB frame
 * @param slot Slot number (0-3)
 * @param slot_type Slot type (0 - signal, 1 - metric, 2 - uio)
 * @param data Slot data (max 240 bytes)
 * @param length Data length
 * @return uint32_t
 */
uint32_t
tio_usb_pack_slot_data(uint8_t slot, uint8_t slot_type, const uint8_t *data, uint32_t length, uint8_t *packet)
{
    if (length > TIO_USB_DATA_LEN)
    {
        ns_lp_printf("Data length exceeds limit\n");
        return 1;
    }
    packet[TIO_USB_START_IDX] = TIO_USB_START_VAL;
    packet[TIO_USB_SLOT_IDX] = slot;
    packet[TIO_USB_TYPE_IDX] = slot_type;
    packet[TIO_USB_DLEN_IDX] = length & 0xFF;
    packet[TIO_USB_DLEN_IDX + 1] = (length >> 8) & 0xFF;
    memcpy(packet + TIO_USB_DATA_IDX, data, length);
    // CRC on data length and data
    uint16_t crc = tio_compute_crc16(packet + TIO_USB_DLEN_IDX, length + TIO_USB_DLEN_LEN);
    packet[TIO_USB_CRC_IDX] = crc & 0xFF;
    packet[TIO_USB_CRC_IDX + 1] = (crc >> 8) & 0xFF;
    packet[TIO_USB_STOP_IDX] = TIO_USB_STOP_VAL;
    return 0;
}

/**
 * @brief Check if USB is mounted and has space to send a packet
 * @return uint32_t
 */
uint32_t
tio_usb_tx_available()
{
    if (!tud_vendor_mounted()) {
        return 0;
    }
    if (tud_vendor_write_available() < TIO_USB_PACKET_LEN) {
        return 0;
    }
    return 1;
}

/**
 * @brief Send packet buffer over USB
 *
 * @param packet USB packet
 * @param length Packet length
 * @return uint32_t
 */
uint32_t
tio_usb_send_slot_packet(uint8_t *packet, uint32_t length)
{
    if (length != TIO_USB_PACKET_LEN)
    {
        ns_lp_printf("Invalid packet length\n");
        return 1;
    }
    if (!tio_usb_tx_available()) {
        return 1;
    }
    webusb_send_data(packet, 256);
    return 0;
}


/**
 * @brief Pack and send slot data
 * @param slot Slot number (0-3)
 * @param slot_type Slot type (0 - signal, 1 - metric, 2 - uio)
 * @param data Slot data (max 240 bytes)
 * @param length Data length
 * @return uint32_t
 */
uint32_t
tio_usb_send_slot_data(uint8_t slot, uint8_t slot_type, const uint8_t *data, uint32_t length)
{
    if (length > TIO_USB_DATA_LEN)
    {
        ns_lp_printf("Data length exceeds limit\n");
        return 1;
    }
    // Send the signal data for given slot
    uint8_t buffer[TIO_USB_PACKET_LEN] = {0};
    buffer[TIO_USB_START_IDX] = TIO_USB_START_VAL;
    buffer[TIO_USB_SLOT_IDX] = slot;
    buffer[TIO_USB_TYPE_IDX] = slot_type;
    buffer[TIO_USB_DLEN_IDX] = length & 0xFF;
    buffer[TIO_USB_DLEN_IDX + 1] = (length >> 8) & 0xFF;
    memcpy(buffer + TIO_USB_DATA_IDX, data, length);
    // CRC on data length and data
    uint16_t crc = tio_compute_crc16(buffer + TIO_USB_DLEN_IDX, length + TIO_USB_DLEN_LEN);
    buffer[TIO_USB_CRC_IDX] = crc & 0xFF;
    buffer[TIO_USB_CRC_IDX + 1] = (crc >> 8) & 0xFF;
    buffer[TIO_USB_STOP_IDX] = TIO_USB_STOP_VAL;
    return tio_usb_send_slot_packet(buffer, TIO_USB_PACKET_LEN);
}

/**
 * @brief Pack and send UIO state
 *
 * @param data
 * @param length
 * @return uint32_t
 */
uint32_t
tio_usb_send_uio_state(const uint8_t *data, uint32_t length)
{
    uint8_t packet[TIO_USB_PACKET_LEN];
    tio_usb_pack_slot_data(0, 2, data, length, packet);
    return tio_usb_send_slot_packet(packet, TIO_USB_PACKET_LEN);
}

/**
 * @brief Initialize the USB system
 *
 * @param ctx Tileio USB context
 * @return uint32_t
 */
uint32_t
tio_usb_init(tio_usb_context_t *ctx)
{
    webusb_register_raw_cb(tio_usb_receive_handler, ctx);

    tio_get_device_id(tioDeviceId);
    tio_device_id_to_serial_id(tioDeviceId, tioSerialId, 13);

    usb_string_desc_arr[USB_DESCRIPTOR_MANUFACTURER] = "Ambiq";
    usb_string_desc_arr[USB_DESCRIPTOR_PRODUCT] = "Tileio";
    usb_string_desc_arr[USB_DESCRIPTOR_SERIAL] = tioSerialId;

    ringbuffer_flush(&tioRxRingBuffer);

    // Initialize USB
    if (ns_usb_init(&tioWebUsbConfig, &tioUsbHandle))
    {
        return 1;
    }
    return 0;
}
