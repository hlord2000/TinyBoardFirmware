#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_vs.h>
#include <zephyr/drivers/sensor.h>
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

/* Notification state */
static bool notifications_enabled = false;
static struct bt_conn *default_conn;

/* Timer for inactivity detection */
static void inactivity_timer_handler(struct k_timer *timer);
K_TIMER_DEFINE(inactivity_timer, inactivity_timer_handler, NULL);

/* Define the accelerometer trigger */
static struct sensor_trigger accel_trigger = {
	.chan = SENSOR_CHAN_ACCEL_XYZ,
	.type = SENSOR_TRIG_THRESHOLD
};

static void accel_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);

BT_GATT_SERVICE_DEFINE(accel_service,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_ACCEL_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_ACCEL_CHAR,
						   BT_GATT_CHRC_NOTIFY,
						   0x00,
						   NULL, NULL, NULL),
	BT_GATT_CCC(accel_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* Send inactivity notification */
static void inactivity_timer_handler(struct k_timer *timer)
{
    if (!notifications_enabled || !default_conn) {
        return;
    }

    uint8_t inactive_buffer[1] = {0}; /* Single byte with value 0 */
    
    LOG_INF("No activity detected for 5 seconds, sending inactive notification");
    
    int err = bt_gatt_notify(default_conn, &accel_service.attrs[1], 
                           inactive_buffer, sizeof(inactive_buffer));
    if (err) {
        LOG_ERR("Failed to send inactivity notification: %d", err);
    }
}

/* Accelerometer trigger handler */
static void accelerometer_trigger_handler(const struct device *dev,
                                        const struct sensor_trigger *trig)
{
    int err = 0;
    struct sensor_value data[3];
    
    LOG_INF("Accelerometer trigger fired! Type: %d", trig->type);
    
    switch (trig->type) {
    case SENSOR_TRIG_MOTION:
    case SENSOR_TRIG_STATIONARY:
    case SENSOR_TRIG_THRESHOLD:
        /* Fetch accelerometer samples */
        if (sensor_sample_fetch(dev) < 0) {
            LOG_ERR("Sample fetch error");
            return;
        }

        /* Get acceleration data */
        err = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, &data[0]);
        if (err) {
            LOG_ERR("sensor_channel_get, error: %d", err);
            return;
        }

        /* Log values for debugging */
        LOG_INF("Activity detected - X: %d.%06d, Y: %d.%06d, Z: %d.%06d", 
                data[0].val1, data[0].val2,
                data[1].val1, data[1].val2,
                data[2].val1, data[2].val2);
        
        /* Send notification if enabled */
        if (notifications_enabled && default_conn) {
            uint8_t buffer[6]; /* X, Y, Z as 16-bit values */
            
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

            /* Reset inactivity timer */
            k_timer_start(&inactivity_timer, K_SECONDS(5), K_NO_WAIT);

            /* Send notification */
            err = bt_gatt_notify(default_conn, &accel_service.attrs[1], buffer, sizeof(buffer));
            if (err) {
                LOG_ERR("Failed to send acceleration notification: %d", err);
            }
        }
        break;
        
    default:
        LOG_ERR("Unknown trigger: %d", trig->type);
    }
}

/* Set activity threshold according to ADXL367 datasheet requirements */
static int configure_activity_threshold(const struct device *dev)
{
    struct sensor_value threshold;
    
    /* Set threshold to approximately 0.5g (4.9 m/s²) */
    /* For 2g range: 0.5g is about 1/4 of full scale */
    threshold.val1 = 4;    /* 4 m/s² */
    threshold.val2 = 903325; /* 4.903325 m/s² total (0.5g) */
    
    LOG_INF("Setting activity threshold to %d.%06d m/s²", 
           threshold.val1, threshold.val2);
    
    /* The driver will convert this value to the 13-bit code needed by the hardware */
    int err = sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ,
                            SENSOR_ATTR_UPPER_THRESH, &threshold);
    if (err) {
        LOG_ERR("Failed to set activity threshold: %d", err);
        return err;
    }
    
    LOG_INF("Activity threshold set successfully");
    return 0;
}

static void start_activity_monitoring(void)
{
    if (!device_is_ready(adxl_dev)) {
        LOG_ERR("ADXL367 device not ready");
        return;
    }
    
    LOG_INF("Starting accelerometer activity monitoring");
    
    /* Configure activity threshold */
    int err = configure_activity_threshold(adxl_dev);
    if (err) {
        LOG_ERR("Failed to configure activity threshold");
        return;
    }
    
    /* Set trigger type to threshold detection */
    accel_trigger.chan = SENSOR_CHAN_ACCEL_XYZ;
    accel_trigger.type = SENSOR_TRIG_THRESHOLD;
    
    err = sensor_trigger_set(adxl_dev, &accel_trigger, accelerometer_trigger_handler);
    if (err) {
        LOG_ERR("Failed to set accelerometer trigger: %d", err);
        return;
    }
    
    LOG_INF("Accelerometer trigger set successfully");
    
    /* Start the inactivity timer */
    k_timer_start(&inactivity_timer, K_SECONDS(5), K_NO_WAIT);
}

static void stop_activity_monitoring(void)
{
    LOG_INF("Stopping accelerometer activity monitoring");
    
    /* Disable the trigger */
    sensor_trigger_set(adxl_dev, &accel_trigger, NULL);
    
    /* Stop the timer */
    k_timer_stop(&inactivity_timer);
}

static void accel_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
	ARG_UNUSED(attr);
	notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
	
	if (notifications_enabled) {
	    LOG_INF("Accel notifications enabled");
	    start_activity_monitoring();
	} else {
	    LOG_INF("Accel notifications disabled");
	    stop_activity_monitoring();
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
        /* Stop accelerometer monitoring if active */
        stop_activity_monitoring();
        notifications_enabled = false;
    }
    
	bt_conn_unref(conn);
	default_conn = NULL;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static int accel_service_init(void) {
	int err;
	
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}
	
	/* Check if the ADXL367 device is ready */
	if (!device_is_ready(adxl_dev)) {
	    LOG_ERR("ADXL367 device not ready");
	    return -ENODEV;
	}
	
	LOG_INF("ADXL367 device is ready");
	
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

SYS_INIT(accel_service_init, APPLICATION, 99);
