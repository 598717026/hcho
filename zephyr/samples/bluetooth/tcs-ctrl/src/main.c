/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 * Copyright (c) 2019 Marcio Montenegro <mtuxpe@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>
#include <drivers/gpio.h>
#include <logging/log.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include "button_svc.h"
#include "led_svc.h"
#include <drivers/i2c.h>

LOG_MODULE_REGISTER(main);

const struct device *i2c0_dev;
const struct device *i2c1_dev;

#define I2C0_DEV "I2C_0"
#define I2C1_DEV "I2C_1"

extern uint16_t but_val;
extern const struct device *led_dev;
extern bool led_state;

/* Prototype */
static ssize_t recv(struct bt_conn *conn,
		    const struct bt_gatt_attr *attr, const void *buf,
		    uint16_t len, uint16_t offset, uint8_t flags);

static ssize_t read_tx(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     void *buf, uint16_t len, uint16_t offset);

/* ST Custom Service  */
static struct bt_uuid_128 st_service_uuid = BT_UUID_INIT_128(
	0x8f, 0xe5, 0xb3, 0xd5, 0x2e, 0x7f, 0x4a, 0x98,
	0x2a, 0x48, 0x7a, 0xcc, 0x40, 0xfe, 0x00, 0x00);

/* ST LED service */
static struct bt_uuid_128 led_char_uuid = BT_UUID_INIT_128(
	0x19, 0xed, 0x82, 0xae, 0xed, 0x21, 0x4c, 0x9d,
	0x41, 0x45, 0x22, 0x8e, 0x41, 0xfe, 0x00, 0x00);

/* ST Notify button service */
static struct bt_uuid_128 but_notif_uuid = BT_UUID_INIT_128(
	0x19, 0xed, 0x82, 0xae, 0xed, 0x21, 0x4c, 0x9d,
	0x41, 0x45, 0x22, 0x8e, 0x42, 0xfe, 0x00, 0x00);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define ADV_LEN 12

/* Advertising data */
static uint8_t manuf_data[ADV_LEN] = {
	0x01 /*SKD version */,
	0x83 /* STM32WB - P2P Server 1 */,
	0x00 /* GROUP A Feature  */,
	0x00 /* GROUP A Feature */,
	0x00 /* GROUP B Feature */,
	0x00 /* GROUP B Feature */,
	0x00, /* BLE MAC start -MSB */
	0x00,
	0x00,
	0x00,
	0x00,
	0x00, /* BLE MAC stop */
};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, manuf_data, ADV_LEN)
};

/* BLE connection */
struct bt_conn *conn;
/* Notification state */
volatile bool notify_enable;

static void mpu_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	notify_enable = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Notification %s", notify_enable ? "enabled" : "disabled");
}

/* The embedded board is acting as GATT server.
 * The ST BLE Android app is the BLE GATT client.
 */

/* ST BLE Sensor GATT services and characteristic */

BT_GATT_SERVICE_DEFINE(stsensor_svc,
BT_GATT_PRIMARY_SERVICE(&st_service_uuid),
BT_GATT_CHARACTERISTIC(&led_char_uuid.uuid,
		       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
		       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, read_tx, recv, (void *)1),
BT_GATT_CHARACTERISTIC(&but_notif_uuid.uuid, BT_GATT_CHRC_NOTIFY,
		       BT_GATT_PERM_READ, NULL, NULL, &but_val),
BT_GATT_CCC(mpu_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static ssize_t recv(struct bt_conn *conn,
		    const struct bt_gatt_attr *attr, const void *buf,
		    uint16_t len, uint16_t offset, uint8_t flags)
{
	if (led_dev) {
		if (led_state) {
			LOG_INF("Turn off LED");
		} else {
			LOG_INF("Turn on LED");
		}
		led_state = !led_state;
		led_on_off(led_state);
	}

	printk("recv:");
	for (size_t i = 0; i < len; i++)
	{
		/* code */
		printk(" %02x ", ((char *)buf)[i]);
	}
	printk("\n");

	return 0;
}

static ssize_t read_tx(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     void *buf, uint16_t len, uint16_t offset)
{
	char buff[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

	return bt_gatt_attr_read(conn, attr, buf, len, offset, buff,
				 sizeof(buff));
}

static void bt_ready(int err)
{
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return;
	}
	LOG_INF("Bluetooth initialized");
	/* Start advertising */
	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}

	LOG_INF("Configuration mode: waiting connections...");
}

static void connected(struct bt_conn *connected, uint8_t err)
{
	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
	} else {
		LOG_INF("Connected");
		if (!conn) {
			conn = bt_conn_ref(connected);
		}
	}
}

static void disconnected(struct bt_conn *disconn, uint8_t reason)
{
	if (conn) {
		bt_conn_unref(conn);
		conn = NULL;
	}

	LOG_INF("Disconnected (reason %u)", reason);
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

#define XY_I2C_ADDR	0x2d

#define Z_I2C_ADDR	0x2d


int i2c_WriteRegXy(uint16_t reg, uint8_t *data, uint32_t num_bytes)
{
	uint8_t adata[50 + 2];

	if (num_bytes > 50)
	{
		return -1;
	}

	adata[0] = ((uint8_t *) &reg)[0];
	adata[1] = ((uint8_t *) &reg)[1];

	int i;
	for (i = 0;i < num_bytes; i++)
	{
		adata[i + 2] = data[i];
	}

	struct i2c_msg msgs[1];

	/* Data to be written, and STOP after this. */
	msgs[0].buf = adata;
	msgs[0].len = num_bytes;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer(i2c1_dev, &msgs[0], 1, XY_I2C_ADDR);
}

int i2c_WriteRegZ(uint16_t reg, uint8_t *data, uint32_t num_bytes)
{
	uint8_t adata[50 + 2];

	if (num_bytes > 50)
	{
		return -1;
	}

	adata[0] = ((uint8_t *) &reg)[0];
	adata[1] = ((uint8_t *) &reg)[1];

	int i;
	for (i = 0;i < num_bytes; i++)
	{
		adata[i + 2] = data[i];
	}

	struct i2c_msg msgs[1];

	/* Data to be written, and STOP after this. */
	msgs[0].buf = adata;
	msgs[0].len = num_bytes;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer(i2c0_dev, &msgs[0], 1, XY_I2C_ADDR);
}

int i2c_ReadRegXy(uint16_t reg, uint8_t *data, uint32_t num_bytes)
{
	uint8_t wr_addr[2];
	struct i2c_msg msgs[2];

	/* Now try to read back from FRAM */

	/* FRAM address */
	wr_addr[0] = (reg >> 8) & 0xFF;
	wr_addr[1] = reg & 0xFF;

	/* Setup I2C messages */

	/* Send the address to read from */
	msgs[0].buf = wr_addr;
	msgs[0].len = 2U;
	msgs[0].flags = I2C_MSG_WRITE;

	/* Read from device. STOP after this. */
	msgs[1].buf = data;
	msgs[1].len = num_bytes;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	return i2c_transfer(i2c1_dev, &msgs[0], 2, XY_I2C_ADDR);
}

int i2c_ReadRegZ(uint16_t reg, uint8_t *data, uint32_t num_bytes)
{
	uint8_t wr_addr[2];
	struct i2c_msg msgs[2];

	/* Now try to read back from FRAM */

	/* FRAM address */
	wr_addr[0] = (reg >> 8) & 0xFF;
	wr_addr[1] = reg & 0xFF;

	/* Setup I2C messages */

	/* Send the address to read from */
	msgs[0].buf = wr_addr;
	msgs[0].len = 2U;
	msgs[0].flags = I2C_MSG_WRITE;

	/* Read from device. STOP after this. */
	msgs[1].buf = data;
	msgs[1].len = num_bytes;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	return i2c_transfer(i2c0_dev, &msgs[0], 2, Z_I2C_ADDR);
}

int i2c_init()
{
	
	i2c0_dev = device_get_binding(I2C0_DEV);
	i2c1_dev = device_get_binding(I2C1_DEV);
	if (!i2c0_dev || !i2c1_dev) {
		printk("I2C: Device driver not found.\n");
		return -1;
	}
	return 0;
}

int ois_off()
{
	int err;
	uint8_t data[] = {0x00};

	printk("ois_off\n");

	err = i2c_WriteRegXy(0x0001 , data, 1);
	if (err)
	{
		printk("i2c_WriteRegXy err:%d\n", err);
		return err;
	}

#if 0
	err = i2c_WriteRegZ(0x0001 , data, 1);
	if (err)
	{
		printk("i2c_WriteRegZ err:%d\n", err);
		return err;
	}
#endif 

	return err;
}

int ois_on()
{
	int err;
	uint8_t data[] = {0x01};

	printk("ois_on\n");

	err = i2c_WriteRegXy(0xf300 , data, 1);
	if (err)
	{
		printk("i2c_WriteRegXy err:%d\n", err);
		return err;
	}

#if 0
	err = i2c_WriteRegZ(0xf300 , data, 1);
	if (err)
	{
		printk("i2c_WriteRegZ err:%d\n", err);
		return err;
	}
#endif

	return err;
}

// 移动x轴 （-1200  -   1200）
int move_x(float s)
{
	int err;

	printk("move_x\n");

	// 寄存器 0xf304～0xf307 为小端，配置数据为float   

	err = i2c_WriteRegXy(0xf304 , ((uint8_t *) &s)[0], 1);
	if (err)
	{
		printk("i2c_WriteRegXy err:%d\n", err);
		return err;
	}

	err = i2c_WriteRegXy(0xf305 , ((uint8_t *) &s)[1], 1);
	if (err)
	{
		printk("i2c_WriteRegXy err:%d\n", err);
		return err;
	}

	err = i2c_WriteRegXy(0xf306 , ((uint8_t *) &s)[2], 1);
	if (err)
	{
		printk("i2c_WriteRegXy err:%d\n", err);
		return err;
	}

	err = i2c_WriteRegXy(0xf307 , ((uint8_t *) &s)[3], 1);
	if (err)
	{
		printk("i2c_WriteRegXy err:%d\n", err);
		return err;
	}
	return err;
}

// 移动y轴 （-1200  -   1200）
int move_y(float s)
{
	int err;
	
	printk("move_y\n");

	// 寄存器 0xf304～0xf307 为小端，配置数据为float   

	err = i2c_WriteRegXy(0xf308 , ((uint8_t *) &s)[0], 1);
	if (err)
	{
		printk("i2c_WriteRegXy err:%d\n", err);
		return err;
	}

	err = i2c_WriteRegXy(0xf309 , ((uint8_t *) &s)[1], 1);
	if (err)
	{
		printk("i2c_WriteRegXy err:%d\n", err);
		return err;
	}

	err = i2c_WriteRegXy(0xf30a , ((uint8_t *) &s)[2], 1);
	if (err)
	{
		printk("i2c_WriteRegXy err:%d\n", err);
		return err;
	}

	err = i2c_WriteRegXy(0xf30b , ((uint8_t *) &s)[3], 1);
	if (err)
	{
		printk("i2c_WriteRegXy err:%d\n", err);
		return err;
	}
	return err;
}


#define IODD_PORT 	15
#define AVDD_PORT 	14
#define VM_PORT 	13

#define GPIO_DRV_NAME DT_LABEL(DT_NODELABEL(gpio0))

#define GPIO_NAME			"GPIO_"

#define GPIO_PIN_WR(dev, pin, bit)						\
	do {									\
		if (gpio_pin_set_raw((dev), (pin), (bit))) {			\
			printk("Err set " GPIO_NAME "%d! %x\n", (pin), (bit));	\
		}								\
	} while (0)								\


#define GPIO_PIN_CFG(dev, pin, dir)						\
	do {									\
		if (gpio_pin_configure((dev), (pin), (dir))) {			\
			printk("Err cfg " GPIO_NAME "%d! %x\n", (pin), (dir));	\
		}								\
	} while (0)

int tcsPowerOn()
{
	int ret = 0;
	const struct device *gpio_dev;

	gpio_dev = device_get_binding(GPIO_DRV_NAME);
	if (!gpio_dev) {
		return (-EOPNOTSUPP);
	}

	//while (1)
	{
		/* code */
	/* Set LED pin as output */

	GPIO_PIN_CFG(gpio_dev, IODD_PORT, GPIO_OUTPUT);
	GPIO_PIN_CFG(gpio_dev, AVDD_PORT, GPIO_OUTPUT);
	GPIO_PIN_CFG(gpio_dev, VM_PORT, GPIO_OUTPUT);

	GPIO_PIN_WR(gpio_dev, IODD_PORT, 0);
	GPIO_PIN_WR(gpio_dev, AVDD_PORT, 0);
	GPIO_PIN_WR(gpio_dev, VM_PORT, 0);

	k_sleep(K_MSEC(300));

	GPIO_PIN_WR(gpio_dev, AVDD_PORT, 1);
	k_sleep(K_MSEC(10));
	GPIO_PIN_WR(gpio_dev, IODD_PORT, 1);
	k_sleep(K_MSEC(10));
	GPIO_PIN_WR(gpio_dev, VM_PORT, 1);

	k_sleep(K_SECONDS(1));
	}
	
	return ret;
}


void main(void)
{
	int err;

	err = i2c_init();
	if (err) {
		return;
	}


	err = tcsPowerOn();
	if (err) {
		return;
	}

	//while(1)
	err = ois_off();
	if (err) {
		return;
	}

	err = ois_on();
	if (err) {
		return;
	}

	err = button_init();
	if (err) {
		return;
	}

	err = led_init();
	if (err) {
		return;
	}
	bt_conn_cb_register(&conn_callbacks);

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(bt_ready);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
	}
	
}
