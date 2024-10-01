/**
 * @file tio_ble.c
 * @author Adam Page (adam.page@ambiq.com)
 * @brief Tileio BLE API
 * @version 0.1
 * @date 2024-10-01
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "arm_math.h"
#include "ns_ambiqsuite_harness.h"
#include "FreeRTOS.h"
#include "task.h"
#include "arm_math.h"
#include "ns_ble.h"

#include "tio_ble.h"

#define TIO_BLE_SLOT_SIG_BUF_LEN (242)
#define TIO_BLE_SLOT_MET_BUF_LEN (242)
#define TIO_BLE_UIO_BUF_LEN (8)

#define TIO_SLOT_SVC_UUID "eecb7db88b2d402cb995825538b49328"
#define TIO_SLOT0_SIG_CHAR_UUID "5bca2754ac7e4a27a1270f328791057a"
#define TIO_SLOT1_SIG_CHAR_UUID "45415793a0e94740bca4ce90bd61839f"
#define TIO_SLOT2_SIG_CHAR_UUID "dd19792c63f1420f920cc58bada8efb9"
#define TIO_SLOT3_SIG_CHAR_UUID "f1f691580bd64cab90a8528baf74cc74"

#define TIO_SLOT0_MET_CHAR_UUID "44a3a7b8d7c849329a10d99dd63775ae"
#define TIO_SLOT1_MET_CHAR_UUID "e64fa683462848c5bede824aaa7c3f5b"
#define TIO_SLOT2_MET_CHAR_UUID "b9d28f5365f04392afbcc602f9dc3c8b"
#define TIO_SLOT3_MET_CHAR_UUID "917c9eb43dbc4cb3bba2ec4e288083f4"

#define TIO_UIO_CHAR_UUID "b9488d48069b47f794f0387f7fbfd1fa"

typedef struct
{
    ns_ble_pool_config_t *pool;
    ns_ble_service_t *service;

    ns_ble_characteristic_t *slot0SigChar;
    ns_ble_characteristic_t *slot1SigChar;
    ns_ble_characteristic_t *slot2SigChar;
    ns_ble_characteristic_t *slot3SigChar;

    ns_ble_characteristic_t *slot0MetChar;
    ns_ble_characteristic_t *slot1MetChar;
    ns_ble_characteristic_t *slot2MetChar;
    ns_ble_characteristic_t *slot3MetChar;

    ns_ble_characteristic_t *uioChar;

    void *slot0SigBuffer;
    void *slot1SigBuffer;
    void *slot2SigBuffer;
    void *slot3SigBuffer;

    void *slot0MetBuffer;
    void *slot1MetBuffer;
    void *slot2MetBuffer;
    void *slot3MetBuffer;

    uint8_t *uioBuffer;

} tio_ble_lcl_context_t;


// WSF buffer pools are a bit of black magic. More development needed.
#define WEBBLE_WSF_BUFFER_POOLS 4
#define WEBBLE_WSF_BUFFER_SIZE \
    (WEBBLE_WSF_BUFFER_POOLS * 16 + 16 * 8 + 32 * 4 + 64 * 6 + 280 * 14) / sizeof(uint32_t)

static uint32_t webbleWSFBufferPool[WEBBLE_WSF_BUFFER_SIZE];
static wsfBufPoolDesc_t webbleBufferDescriptors[WEBBLE_WSF_BUFFER_POOLS] = {
    {16, 8}, // 16 bytes, 8 buffers
    {32, 4},
    {64, 6},
    {512, 14}}; // 512

static ns_ble_pool_config_t bleWsfBuffers = {
    .pool = webbleWSFBufferPool,
    .poolSize = sizeof(webbleWSFBufferPool),
    .desc = webbleBufferDescriptors,
    .descNum = WEBBLE_WSF_BUFFER_POOLS};

static uint8_t bleSlot0SigBuffer[TIO_BLE_SLOT_SIG_BUF_LEN] = {0};
static uint8_t bleSlot1SigBuffer[TIO_BLE_SLOT_SIG_BUF_LEN] = {0};
static uint8_t bleSlot2SigBuffer[TIO_BLE_SLOT_SIG_BUF_LEN] = {0};
static uint8_t bleSlot3SigBuffer[TIO_BLE_SLOT_SIG_BUF_LEN] = {0};
static uint8_t *bleSlotBuffers[4] = {
    bleSlot0SigBuffer,
    bleSlot1SigBuffer,
    bleSlot2SigBuffer,
    bleSlot3SigBuffer};
static uint8_t bleSlot0MetBuffer[TIO_BLE_SLOT_MET_BUF_LEN] = {0};
static uint8_t bleSlot1MetBuffer[TIO_BLE_SLOT_MET_BUF_LEN] = {0};
static uint8_t bleSlot2MetBuffer[TIO_BLE_SLOT_MET_BUF_LEN] = {0};
static uint8_t bleSlot3MetBuffer[TIO_BLE_SLOT_MET_BUF_LEN] = {0};
static uint8_t *bleSlotMetBuffers[4] = {
    bleSlot0MetBuffer,
    bleSlot1MetBuffer,
    bleSlot2MetBuffer,
    bleSlot3MetBuffer};
static uint8_t bleUioBuffer[TIO_BLE_UIO_BUF_LEN] = {0};

static ns_ble_service_t bleService;
static ns_ble_characteristic_t bleSlot0SigChar;
static ns_ble_characteristic_t bleSlot1SigChar;
static ns_ble_characteristic_t bleSlot2SigChar;
static ns_ble_characteristic_t bleSlot3SigChar;
static ns_ble_characteristic_t bleSlot0MetChar;
static ns_ble_characteristic_t bleSlot1MetChar;
static ns_ble_characteristic_t bleSlot2MetChar;
static ns_ble_characteristic_t bleSlot3MetChar;
static ns_ble_characteristic_t *bleSlotSigChars[4] = {
    &bleSlot0SigChar,
    &bleSlot1SigChar,
    &bleSlot2SigChar,
    &bleSlot3SigChar};
static ns_ble_characteristic_t *bleSlotMetChars[4] = {
    &bleSlot0MetChar,
    &bleSlot1MetChar,
    &bleSlot2MetChar,
    &bleSlot3MetChar};

static ns_ble_characteristic_t bleUioChar;

static tio_ble_lcl_context_t tioBleCtx = {
    .pool = &bleWsfBuffers,
    .service = &bleService,
    .slot0SigChar = &bleSlot0SigChar,
    .slot1SigChar = &bleSlot1SigChar,
    .slot2SigChar = &bleSlot2SigChar,
    .slot3SigChar = &bleSlot3SigChar,
    .slot0MetChar = &bleSlot0MetChar,
    .slot1MetChar = &bleSlot1MetChar,
    .slot2MetChar = &bleSlot2MetChar,
    .slot3MetChar = &bleSlot3MetChar,
    .uioChar = &bleUioChar,
    .slot0SigBuffer = bleSlot0SigBuffer,
    .slot1SigBuffer = bleSlot1SigBuffer,
    .slot2SigBuffer = bleSlot2SigBuffer,
    .slot3SigBuffer = bleSlot3SigBuffer,
    .slot0MetBuffer = bleSlot0MetBuffer,
    .slot1MetBuffer = bleSlot1MetBuffer,
    .slot2MetBuffer = bleSlot2MetBuffer,
    .slot3MetBuffer = bleSlot3MetBuffer,
    .uioBuffer = bleUioBuffer
};

static tio_ble_context_t *gTioBleCtx = NULL;

void
webbleHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
    ns_lp_printf("webbleHandler\n");
}
void
webbleHandlerInit(wsfHandlerId_t handlerId)
{
    ns_lp_printf("webbleHandlerInit\n");
}

int
tio_ble_notify_sig_handler(ns_ble_service_t *s, struct ns_ble_characteristic *c)
{
    return NS_STATUS_SUCCESS;
}

int
tio_ble_notify_met_handler(ns_ble_service_t *s, struct ns_ble_characteristic *c)
{
    return NS_STATUS_SUCCESS;
}

int
tio_ble_notify_uio_handler(ns_ble_service_t *s, struct ns_ble_characteristic *c)
{
    return NS_STATUS_SUCCESS;
}

int
tio_ble_uio_read_handler(ns_ble_service_t *s, struct ns_ble_characteristic *c, void *dest)
{
    memcpy(dest, c->applicationValue, c->valueLen);
    return NS_STATUS_SUCCESS;
}

int
tio_ble_uio_write_handler(ns_ble_service_t *s, struct ns_ble_characteristic *c, void *src)
{
    memcpy(c->applicationValue, src, c->valueLen);
    if (c == tioBleCtx.uioChar)
    {
        if (gTioBleCtx->uio_update_cb != NULL)
        {
            gTioBleCtx->uio_update_cb(src, c->valueLen);
        }
    }
    return NS_STATUS_SUCCESS;
}

void
tio_ble_send_slot_data(uint8_t slot, uint8_t slot_type, const uint8_t *data, uint32_t length)
{
    uint8_t *buffer = NULL;
    ns_ble_characteristic_t *bleChar = NULL;
    if (length > 240)
    {
        ns_lp_printf("Data length exceeds 240 bytes\n");
        return;
    }
    if (slot >= 4)
    {
        ns_lp_printf("Invalid slot number\n");
        return;
    }
    if (slot_type >= 2)
    {
        ns_lp_printf("Invalid slot type\n");
        return;
    }
    if (slot_type == 0)
    {
        buffer = bleSlotBuffers[slot];
        bleChar = bleSlotSigChars[slot];
    }
    else
    {
        buffer = bleSlotMetBuffers[slot];
        bleChar = bleSlotMetChars[slot];
    }
    buffer[0] = length & 0xFF;
    buffer[1] = (length >> 8) & 0xFF;
    memset(buffer + 2, 0, 240);
    memcpy(buffer + 2, data, length);
    ns_ble_send_value(bleChar, NULL);
}

void
tio_ble_send_uio_state(const uint8_t *data, uint32_t length)
{
    if (length != 8)
    {
        ns_lp_printf("Invalid UIO data length\n");
        return;
    }
    memcpy(tioBleCtx.uioBuffer, data, length);
    ns_ble_send_value(tioBleCtx.uioChar, NULL);
}

static int
tio_ble_service_init(void)
{
    // Initialize BLE service
    char bleName[] = "Tileio"; // TODO: Get custom name
    NS_TRY(ns_ble_char2uuid(TIO_SLOT_SVC_UUID, &(tioBleCtx.service->uuid128)), "Failed to convert UUID\n");
    memcpy(tioBleCtx.service->name, bleName, sizeof(bleName));
    tioBleCtx.service->nameLen = strlen(bleName);
    tioBleCtx.service->baseHandle = 0x0800;
    tioBleCtx.service->poolConfig = tioBleCtx.pool;
    tioBleCtx.service->numAttributes = 0;

    // Create all slots
    ns_ble_create_characteristic(
        tioBleCtx.slot0SigChar, TIO_SLOT0_SIG_CHAR_UUID, tioBleCtx.slot0SigBuffer, TIO_BLE_SLOT_SIG_BUF_LEN,
        NS_BLE_READ | NS_BLE_NOTIFY,
        NULL, NULL, &tio_ble_notify_sig_handler,
        1000, true, &(tioBleCtx.service->numAttributes));
    ns_ble_create_characteristic(
        tioBleCtx.slot0MetChar, TIO_SLOT0_MET_CHAR_UUID, tioBleCtx.slot0MetBuffer, TIO_BLE_SLOT_MET_BUF_LEN,
        NS_BLE_READ | NS_BLE_NOTIFY,
        NULL, NULL, &tio_ble_notify_met_handler,
        1000, true, &(tioBleCtx.service->numAttributes));

    ns_ble_create_characteristic(
        tioBleCtx.slot1SigChar, TIO_SLOT1_SIG_CHAR_UUID, tioBleCtx.slot1SigBuffer, TIO_BLE_SLOT_SIG_BUF_LEN,
        NS_BLE_READ | NS_BLE_NOTIFY,
        NULL, NULL, &tio_ble_notify_sig_handler,
        1000, true, &(tioBleCtx.service->numAttributes));
    ns_ble_create_characteristic(
        tioBleCtx.slot1MetChar, TIO_SLOT1_MET_CHAR_UUID, tioBleCtx.slot1MetBuffer, TIO_BLE_SLOT_MET_BUF_LEN,
        NS_BLE_READ | NS_BLE_NOTIFY,
        NULL, NULL, &tio_ble_notify_met_handler,
        1000, true, &(tioBleCtx.service->numAttributes));

    ns_ble_create_characteristic(
        tioBleCtx.slot2SigChar, TIO_SLOT2_SIG_CHAR_UUID, tioBleCtx.slot2SigBuffer, TIO_BLE_SLOT_SIG_BUF_LEN,
        NS_BLE_READ | NS_BLE_NOTIFY,
        NULL, NULL, &tio_ble_notify_sig_handler,
        1000, true, &(tioBleCtx.service->numAttributes));
    ns_ble_create_characteristic(
        tioBleCtx.slot2MetChar, TIO_SLOT2_MET_CHAR_UUID, tioBleCtx.slot2MetBuffer, TIO_BLE_SLOT_MET_BUF_LEN,
        NS_BLE_READ | NS_BLE_NOTIFY,
        NULL, NULL, &tio_ble_notify_met_handler,
        1000, true, &(tioBleCtx.service->numAttributes));

    ns_ble_create_characteristic(
        tioBleCtx.slot3SigChar, TIO_SLOT3_SIG_CHAR_UUID, tioBleCtx.slot3SigBuffer, TIO_BLE_SLOT_SIG_BUF_LEN,
        NS_BLE_READ | NS_BLE_NOTIFY,
        NULL, NULL, &tio_ble_notify_sig_handler,
        1000, true, &(tioBleCtx.service->numAttributes));
    ns_ble_create_characteristic(
        tioBleCtx.slot3MetChar, TIO_SLOT3_MET_CHAR_UUID, tioBleCtx.slot3MetBuffer, TIO_BLE_SLOT_MET_BUF_LEN,
        NS_BLE_READ | NS_BLE_NOTIFY,
        NULL, NULL, tio_ble_notify_met_handler,
        1000, true, &(tioBleCtx.service->numAttributes));

    // UIO
    ns_ble_create_characteristic(
        tioBleCtx.uioChar, TIO_UIO_CHAR_UUID, tioBleCtx.uioBuffer, TIO_BLE_UIO_BUF_LEN,
        NS_BLE_READ | NS_BLE_WRITE | NS_BLE_NOTIFY,
        &tio_ble_uio_read_handler, &tio_ble_uio_write_handler, &tio_ble_notify_uio_handler,
        1000, true, &(tioBleCtx.service->numAttributes));

    tioBleCtx.service->numCharacteristics = 9;
    ns_ble_create_service(tioBleCtx.service);
    ns_ble_add_characteristic(tioBleCtx.service, tioBleCtx.slot0SigChar);
    ns_ble_add_characteristic(tioBleCtx.service, tioBleCtx.slot0MetChar);
    ns_ble_add_characteristic(tioBleCtx.service, tioBleCtx.slot1SigChar);
    ns_ble_add_characteristic(tioBleCtx.service, tioBleCtx.slot1MetChar);
    ns_ble_add_characteristic(tioBleCtx.service, tioBleCtx.slot2SigChar);
    ns_ble_add_characteristic(tioBleCtx.service, tioBleCtx.slot2MetChar);
    ns_ble_add_characteristic(tioBleCtx.service, tioBleCtx.slot3SigChar);
    ns_ble_add_characteristic(tioBleCtx.service, tioBleCtx.slot3MetChar);
    ns_ble_add_characteristic(tioBleCtx.service, tioBleCtx.uioChar);
    // Initialize BLE, create structs, start service
    ns_ble_start_service(tioBleCtx.service);
    return NS_STATUS_SUCCESS;
}

void TioBleTask(void *pvParameters)
{
    NS_TRY(tio_ble_service_init(), "BLE init failed.\n");
    while (1)
    {
        wsfOsDispatcher();
    }
}

uint32_t
tio_ble_init(tio_context_t *ctx)
{
    gTioBleCtx = ctx;
    ns_ble_pre_init();
    return NS_STATUS_SUCCESS;
}
