/**
 * @file ns_tileio.h
 * @author Adam Page (adam.page@ambiq.com)
 * @brief Tileio USB API
 * @version 0.1
 * @date 2024-08-30
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef __TIO_USB_H
#define __TIO_USB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "arm_math.h"

#define TIO_USB_PACKET_LEN 256


// A USB slot frame is 256 bytes long w/ fields:
//   START: 1 byte      [0x55]
//    SLOT: 1 byte      [0 - ch0, 1 - ch1, 2 - ch2, 3 - ch3]
//   STYPE: 1 byte      [0 - signal, 1 - metric, 2 - uio]
//  LENGTH: 2 bytes     [0 - 248]
//    DATA: 248 bytes   [...]
//     CRC: 2 bytes     [CRC16]
//    STOP: 1 byte      [0xAA]

typedef void (*pfnSlotUpdate)(uint8_t slot, uint8_t slot_type, const uint8_t *data, uint32_t length);
typedef void (*pfnUioUpdate)(const uint8_t *data, uint32_t length);

typedef struct {
    volatile pfnUioUpdate uio_update_cb;
    volatile pfnSlotUpdate slot_update_cb;
} tio_usb_context_t;

uint32_t
tio_usb_init(tio_usb_context_t *ctx);
uint32_t
tio_usb_tx_available();
uint32_t
tio_usb_pack_slot_data(uint8_t slot, uint8_t slot_type, const uint8_t *data, uint32_t length, uint8_t *packet);
uint32_t
tio_usb_send_slot_packet(uint8_t *buffer, uint32_t length);
uint32_t
tio_usb_send_slot_data(uint8_t slot, uint8_t slot_type, const uint8_t *data, uint32_t length);
uint32_t
tio_usb_send_uio_state(const uint8_t *data, uint32_t length);


#ifdef __cplusplus
}
#endif

#endif // __TIO_USB_H
