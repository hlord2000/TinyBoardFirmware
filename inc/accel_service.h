#ifndef ACCEL_SERVICE_H__
#define ACCEL_SERVICE_H__

#include <zephyr/types.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#define BT_UUID_ACCEL_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x0901BB00, 0xE287, 0x4548, 0xBA91, 0x3A92B284F904)
#define BT_UUID_ACCEL_CHAR_VAL \
    BT_UUID_128_ENCODE(0x0901BB01, 0xE287, 0x4548, 0xBA91, 0x3A92B284F904)
#define BT_UUID_ACCEL_SERVICE           BT_UUID_DECLARE_128(BT_UUID_ACCEL_SERVICE_VAL)
#define BT_UUID_ACCEL_CHAR              BT_UUID_DECLARE_128(BT_UUID_ACCEL_CHAR_VAL)

#endif
