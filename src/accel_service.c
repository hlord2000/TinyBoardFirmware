#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_vs.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/led.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include "accel_service.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(accel_service, LOG_LEVEL_DBG);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_ACCEL_SERVICE_VAL),
};

/* ADXL367 device */
static const struct device *adxl_dev = DEVICE_DT_GET(DT_NODELABEL(adxl366));

/* LED Device */
#define LED_PWM_NODE_ID	 DT_COMPAT_GET_ANY_STATUS_OKAY(pwm_leds)
static const struct device *led_dev = DEVICE_DT_GET(LED_PWM_NODE_ID);

const char *led_label[] = {
	DT_FOREACH_CHILD_SEP_VARGS(LED_PWM_NODE_ID, DT_PROP_OR, (,), label, NULL)
};

const int num_leds = ARRAY_SIZE(led_label);

/* LED configuration */
#define MAX_BRIGHTNESS	100
#define FADE_DELAY_MS	10
#define NOTIFICATION_LED 0  /* Use the first LED for notification indication */

/* Notification state */
static bool notifications_enabled = false;
static struct bt_conn *default_conn;

/* Timer for sensor polling - 10 Hz (100ms interval) */
#define POLLING_INTERVAL_MS 100
static void polling_timer_handler(struct k_timer *timer);
K_TIMER_DEFINE(polling_timer, polling_timer_handler, NULL);

/* Work item for sensor reading */
static struct k_work sensor_work;
static void sensor_work_handler(struct k_work *work);

/* Work item for LED control */
static struct k_work led_work;
static void led_work_handler(struct k_work *work);

static void accel_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);

BT_GATT_SERVICE_DEFINE(accel_service,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_ACCEL_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_ACCEL_CHAR,
						   BT_GATT_CHRC_NOTIFY,
						   0x00,
						   NULL, NULL, NULL),
	BT_GATT_CCC(accel_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* LED control function - pulse when notifications are enabled */
static void led_work_handler(struct k_work *work)
{
    static bool led_increasing = true;
    static uint16_t led_level = 0;
    int err;

    if (!device_is_ready(led_dev) || NOTIFICATION_LED >= num_leds) {
        return;
    }

    if (notifications_enabled) {
        /* Create a breathing effect when notifications are enabled */
        if (led_increasing) {
            led_level += 5;
            if (led_level >= MAX_BRIGHTNESS) {
                led_level = MAX_BRIGHTNESS;
                led_increasing = false;
            }
        } else {
            if (led_level <= 5) {
                led_level = 0;
                led_increasing = true;
            } else {
                led_level -= 5;
            }
        }

        err = led_set_brightness(led_dev, NOTIFICATION_LED, led_level);
        if (err < 0) {
            LOG_ERR("Failed to set LED brightness: %d", err);
        }

        /* Schedule next update */
        k_work_submit(&led_work);
    } else {
        /* Turn LED off when notifications are disabled */
        err = led_off(led_dev, NOTIFICATION_LED);
        if (err < 0) {
            LOG_ERR("Failed to turn LED off: %d", err);
        }
    }
}

/* Sensor polling timer handler */
static void polling_timer_handler(struct k_timer *timer)
{
    /* Submit work to the system work queue */
    k_work_submit(&sensor_work);
}

/* Sensor work handler */
static void sensor_work_handler(struct k_work *work)
{
    int err = 0;
    struct sensor_value data[3];
    uint8_t buffer[6]; /* X, Y, Z as 16-bit values */
    
    /* Only proceed if notifications are enabled and connected */
    if (!notifications_enabled || !default_conn) {
        return;
    }
    
    /* Fetch accelerometer samples */
    if (sensor_sample_fetch(adxl_dev) < 0) {
        LOG_ERR("Sample fetch error");
        return;
    }

    /* Get acceleration data */
    err = sensor_channel_get(adxl_dev, SENSOR_CHAN_ACCEL_XYZ, &data[0]);
    if (err) {
        LOG_ERR("sensor_channel_get, error: %d", err);
        return;
    }

    /* Debug log */
    LOG_DBG("X: %d.%06d, Y: %d.%06d, Z: %d.%06d", 
            data[0].val1, data[0].val2,
            data[1].val1, data[1].val2,
            data[2].val1, data[2].val2);
    
    /* Convert to int16_t values (in milli-g) for BLE transmission */
    int16_t x_int = (int16_t)(sensor_value_to_double(&data[0]) * 1000);
    int16_t y_int = (int16_t)(sensor_value_to_double(&data[1]) * 1000);
    int16_t z_int = (int16_t)(sensor_value_to_double(&data[2]) * 1000);

    /* Pack into buffer (little-endian) */
    buffer[0] = x_int & 0xFF;
    buffer[1] = (x_int >> 8) & 0xFF;
    buffer[2] = y_int & 0xFF;
    buffer[3] = (y_int >> 8) & 0xFF;
    buffer[4] = z_int & 0xFF;
    buffer[5] = (z_int >> 8) & 0xFF;

    /* Send notification */
    err = bt_gatt_notify(default_conn, &accel_service.attrs[1], buffer, sizeof(buffer));
    if (err) {
        LOG_ERR("Failed to send acceleration notification: %d", err);
    }
}

static void start_sensor_polling(void)
{
    if (!device_is_ready(adxl_dev)) {
        LOG_ERR("ADXL367 device not ready");
        return;
    }
    
    LOG_INF("Starting accelerometer polling at 10Hz");
    
    /* Start the polling timer */
    k_timer_start(&polling_timer, K_MSEC(POLLING_INTERVAL_MS), K_MSEC(POLLING_INTERVAL_MS));
    
    /* Start LED breathing effect */
    if (device_is_ready(led_dev) && NOTIFICATION_LED < num_leds) {
        k_work_submit(&led_work);
    }
}

static void stop_sensor_polling(void)
{
    LOG_INF("Stopping accelerometer polling");
    
    /* Stop the polling timer */
    k_timer_stop(&polling_timer);
    
    /* Turn off LED */
    if (device_is_ready(led_dev) && NOTIFICATION_LED < num_leds) {
        led_off(led_dev, NOTIFICATION_LED);
    }
}

static void accel_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
	ARG_UNUSED(attr);
	notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
	
	if (notifications_enabled) {
	    LOG_INF("Accel notifications enabled");
	    start_sensor_polling();
	} else {
	    LOG_INF("Accel notifications disabled");
	    stop_sensor_polling();
	}
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err != 0) {
		return;
	}
	default_conn = bt_conn_ref(conn);
	LOG_INF("Device connected");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Device disconnected (reason: %u)", reason);
    
    if (notifications_enabled) {
        /* Stop sensor polling if active */
        stop_sensor_polling();
        notifications_enabled = false;
    }
    
	bt_conn_unref(conn);
	default_conn = NULL;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static int accel_service_init(void)
{
	int err;
	
	/* Initialize work items */
	k_work_init(&sensor_work, sensor_work_handler);
	k_work_init(&led_work, led_work_handler);
	
	/* Delay startup to let other components initialize */
	k_msleep(5000);
	
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}
	
	/* Check if the ADXL367 device is ready */
	if (!device_is_ready(adxl_dev)) {
	    LOG_ERR("ADXL367 device not ready");
	    return -ENODEV;
	}
	
	LOG_INF("ADXL367 device is ready");
	
	/* Check if LED device is ready */
	if (!device_is_ready(led_dev)) {
	    LOG_WRN("LED device not ready");
	} else if (num_leds > 0) {
	    LOG_INF("LED device ready with %d LEDs", num_leds);
	    
	    /* Turn LED off initially */
	    if (NOTIFICATION_LED < num_leds) {
	        led_off(led_dev, NOTIFICATION_LED);
	    }
	}
	
	err = bt_enable(NULL);
	if (err < 0) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return err;
	}
	
	struct bt_le_adv_param adv_param = {
		.id = BT_ID_DEFAULT,
		.sid = 0,
		.secondary_max_skip = 0,
		.options = (BT_LE_ADV_OPT_CONNECTABLE |
			    BT_LE_ADV_OPT_ONE_TIME),
		.interval_min = BT_GAP_ADV_SLOW_INT_MIN,
		.interval_max = BT_GAP_ADV_SLOW_INT_MAX,
		.peer = NULL,
	};
	
	err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err < 0) {
		LOG_ERR("Bluetooth advertising start failed (err %d)", err);
		return err;
	}
	
	LOG_INF("Bluetooth advertising started");
	return 0;
}

/* Initialize as application with high priority (99) */
SYS_INIT(accel_service_init, APPLICATION, 99);
