/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
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
#include <logging/log.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/services/bas.h>
#include <bluetooth/services/hrs.h>


LOG_MODULE_REGISTER(main);

static  int16_t m_adc_sample = 0;
static  int16_t m_hcho_ppb = 0;



static void dump_buffer(uint8_t *buf, size_t size)
{
	bool newline = false;
	uint8_t *p = buf;

	while (size >= 16) {
		printk("%02x %02x %02x %02x | %02x %02x %02x %02x |" \
		       "%02x %02x %02x %02x | %02x %02x %02x %02x\n",
		       p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7],
			   p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15]);
		p += 16;
		size -= 16;
	}
	if (size >= 8) {
		printk("%02x %02x %02x %02x | %02x %02x %02x %02x\n",
		       p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
		p += 8;
		size -= 8;
		newline = true;
	}
	if (size > 4) {
		printk("%02x %02x %02x %02x | ",
		       p[0], p[1], p[2], p[3]);
		p += 4;
		size -= 4;
		newline = true;
	}
	while (size--) {
		printk("%02x ", *p++);
		newline = true;
	}
	if (newline) {
		printk("\n");
	}
}






#if 1

#include <drivers/flash.h>


static const struct device *flash_device;

void do_flash_init(void)
{
	flash_device =
		device_get_binding(DT_CHOSEN_ZEPHYR_FLASH_CONTROLLER_LABEL);
	if (flash_device) {
		printk("Found flash controller %s.\n",
			DT_CHOSEN_ZEPHYR_FLASH_CONTROLLER_LABEL);
		printk("Flash I/O commands can be run.\n");
	} else {
		printk("**No flash controller found!**\n");
		printk("Run set_device <name> to specify one "
		       "before using other commands.\n");
	}
}

/* Read bytes, dumping contents to console and printing on error. */
static int do_read(off_t offset, size_t len, uint8_t *buf)
{
	int ret;

	/*while (len > sizeof(buf)) {
		ret = flash_read(flash_device, offset, buf, sizeof(buf));
		if (ret) {
			goto err_read;
		}
		len -= sizeof(buf);
		offset += sizeof(buf);
	}*/
	ret = flash_read(flash_device, offset, buf, len);
	if (ret) {
		goto err_read;
	}
	return 0;

 err_read:
	printk("flash_read error: %d\n", ret);
	return ret;
}

/* Erase area, handling write protection and printing on error. */
static int do_erase(off_t offset, size_t size)
{
	int ret;

	ret = flash_write_protection_set(flash_device, false);
	if (ret) {
		printk("Failed to disable flash protection (err: %d)."
				"\n", ret);
		return ret;
	}
	ret = flash_erase(flash_device, offset, size);
	if (ret) {
		printk("flash_erase failed (err:%d).\n", ret);
		return ret;
	}
	ret = flash_write_protection_set(flash_device, true);
	if (ret) {
		printk("Failed to enable flash protection (err: %d)."
				"\n", ret);
	}
	return ret;
}

/* Write bytes, handling write protection and printing on error. */
static int do_write(off_t offset, uint8_t *buf,
		    size_t len, bool read_back)
{
	int ret;

	ret = flash_write_protection_set(flash_device, false);
	if (ret) {
		printk("Failed to disable flash protection (err: %d)."
				"\n", ret);
		return ret;
	}
	ret = flash_write(flash_device, offset, buf, len);
	if (ret) {
		printk("flash_write failed (err:%d).\n", ret);
		return ret;
	}
	ret = flash_write_protection_set(flash_device, true);
	if (ret) {
		printk("Failed to enable flash protection (err: %d)."
				"\n", ret);
		return ret;
	}
	if (read_back) {
		printk("Reading back written bytes:\n");
		ret = do_read(offset, len, buf);
	}
	return ret;
}


#define	SETTING_ADDR	520192
#define	PAGE_SIZE		4096

static uint8_t gSettingppbz[2];
static uint8_t gSettingppbh[2];
static uint8_t gSettingsamplez[2];
static uint8_t gSettingsampleh[2];
static uint8_t gMode;
static uint8_t gSn[5];

static bool isUpdate = false;

void readSettings(uint8_t *ppbz, uint8_t *ppbh, uint8_t *samplez, uint8_t *sampleh, uint8_t *mode, uint8_t *sn)
{
		ppbz[0] = gSettingppbz[0];
		ppbz[1] = gSettingppbz[1];
		ppbh[0] = gSettingppbh[0];
		ppbh[1] = gSettingppbh[1];
		samplez[0] = gSettingsamplez[0];
		samplez[1] = gSettingsamplez[1];
		sampleh[0] = gSettingsampleh[0];
		sampleh[1] = gSettingsampleh[1];
		*mode = gMode;
		sn[0] = gSn[0];
		sn[1] = gSn[1];
		sn[2] = gSn[2];
		sn[3] = gSn[3];
		sn[4] = gSn[4];
}

void doReadSettings()
{
	int ret;
	uint8_t pbuf[14];

	ret = do_read(SETTING_ADDR, 14, pbuf);
	
	dump_buffer(pbuf, 14);
	if (0 == ret)
	{
		gSettingppbz[0] = pbuf[0];
		gSettingppbz[1] = pbuf[1];
		gSettingppbh[0] = pbuf[2];
		gSettingppbh[1] = pbuf[3];
		gSettingsamplez[0] = pbuf[4];
		gSettingsamplez[1] = pbuf[5];
		gSettingsampleh[0] = pbuf[6];
		gSettingsampleh[1] = pbuf[7];
		gMode = pbuf[8];
		//gMode = 0xff;
		gSn[0] = pbuf[9];
		gSn[1] = pbuf[10];
		gSn[2] = pbuf[11];
		gSn[3] = pbuf[12];
		gSn[4] = pbuf[13];
	}
}

void writeSettings(uint8_t *ppbz, uint8_t *ppbh, uint8_t *samplez, uint8_t *sampleh, uint8_t mode, uint8_t *sn)
{
	//printk("writeSettings:%02x%02x%02x%02x%02x%02x%02x%02x\n", ppbz[0], ppbz[1], ppbh[0], ppbh[1], samplez[0], samplez[1], sampleh[0], sampleh[1]);
		gSettingppbz[0] = ppbz[0];
		gSettingppbz[1] = ppbz[1];
		gSettingppbh[0] = ppbh[0];
		gSettingppbh[1] = ppbh[1];
		gSettingsamplez[0] = samplez[0];
		gSettingsamplez[1] = samplez[1];
		gSettingsampleh[0] = sampleh[0];
		gSettingsampleh[1] = sampleh[1];
		gMode = mode;
		gSn[0] = sn[0];
		gSn[1] = sn[1];
		gSn[2] = sn[2];
		gSn[3] = sn[3];
		gSn[4] = sn[4];
	//printk("writeSettings xx:%02x%02x%02x%02x%02x%02x%02x%02x\n", gSettingppbz[0], gSettingppbz[1], gSettingppbh[0], gSettingppbh[1], gSettingsamplez[0], gSettingsamplez[1], gSettingsampleh[0], gSettingsampleh[1]);
		isUpdate = true;
}

void getWriteSettingsBuffer(uint8_t *buf)
{

	buf[0] = gSettingppbz[0];
	buf[1] = gSettingppbz[1];
	buf[2] = gSettingppbh[0];
	buf[3] = gSettingppbh[1];
	buf[4] = gSettingsamplez[0];
	buf[5] = gSettingsamplez[1];
	buf[6] = gSettingsampleh[0];
	buf[7] = gSettingsampleh[1];
	buf[8] = gMode;
	buf[9] = gSn[0];
	buf[10] = gSn[1];
	buf[11] = gSn[2];
	buf[12] = gSn[3];
	buf[13] = gSn[4];

}

void setWriteSettingsBuffer(uint8_t *buf)
{

		gSettingppbz[0] = buf[0];
		gSettingppbz[1] = buf[1];
		gSettingppbh[0] = buf[2];
		gSettingppbh[1] = buf[3];
		gSettingsamplez[0] = buf[4];
		gSettingsamplez[1] = buf[5];
		gSettingsampleh[0] = buf[6];
		gSettingsampleh[1] = buf[7];
		gMode = buf[8];
		//gMode = 0xff;
		gSn[0] = buf[9];
		gSn[1] = buf[10];
		gSn[2] = buf[11];
		gSn[3] = buf[12];
		gSn[4] = buf[13];
}

void doWriteSettings()
{
	uint8_t pbuf[14];
	do_erase(SETTING_ADDR, PAGE_SIZE);
	
	pbuf[0] = gSettingppbz[0];
	pbuf[1] = gSettingppbz[1];
	pbuf[2] = gSettingppbh[0];
	pbuf[3] = gSettingppbh[1];
	pbuf[4] = gSettingsamplez[0];
	pbuf[5] = gSettingsamplez[1];
	pbuf[6] = gSettingsampleh[0];
	pbuf[7] = gSettingsampleh[1];
	pbuf[8] = gMode;
	pbuf[9] = gSn[0];
	pbuf[10] = gSn[1];
	pbuf[11] = gSn[2];
	pbuf[12] = gSn[3];
	pbuf[13] = gSn[4];

	//printk("doWriteSettings:%02x%02x%02x%02x%02x%02x%02x%02x\n", pbuf[0], pbuf[1], pbuf[2], pbuf[3], pbuf[4], pbuf[5], pbuf[6], pbuf[7]);
	//k_sleep(K_SECONDS(1));
	do_write(SETTING_ADDR, pbuf, 14, false);
}

static struct k_thread settingsUpdate_thread_data;
static K_THREAD_STACK_DEFINE(settingsUpdate_thread_stack, 320);

void modulate_settingsUpdate(void *p1, void *p2, void *p3)
{
	do_flash_init();
	doReadSettings();
	//isUpdate = true;
	k_sleep(K_SECONDS(3));
	while (1) {
		if (isUpdate)
		{
			doWriteSettings();
			//doReadSettings();
			isUpdate = false;
		}
		autoUpLoadV();
		k_sleep(K_SECONDS(1));
	}
}

#endif


#if 1

#include <drivers/uart.h>

#define UART_DEVICE_NAME CONFIG_UART_CONSOLE_ON_DEV_NAME

static volatile bool data_transmitted;
static volatile bool data_received;
static int char_sent;
static const char fifo_data[] = "This is a FIFO test.\r\n";
static struct device *uart_dev;

#define DATA_SIZE	(sizeof(fifo_data) - 1)

uint8_t commandCheckSum(uint8_t *cmdbuffer)
{
	uint8_t checksum = 0;
	for (int i = 1; i <= 7; i++)
	{
		checksum += cmdbuffer[i];
	}
	checksum = (~checksum) + 1;
	return checksum;
}

static void uartPutChar(uint8_t d)
{
	z_impl_uart_poll_out(uart_dev, d);
}

void sampleToV(uint16_t s, uint16_t *c, uint16_t *ppb)
{
#if 0
	uint8_t ppbz[2];
	uint8_t ppbh[2];
	uint8_t samplez[2];
	uint8_t sampleh[2];
	uint8_t mode;
	uint8_t sn[5];

	readSettings(ppbz, ppbh, samplez, sampleh, &mode, sn);

	uint16_t ppbzv = ppbz[0] * 256 + ppbz[1];
	uint16_t ppbhv = ppbh[0] * 256 + ppbh[1];
	uint16_t samplezv = samplez[0] * 256 + samplez[1];
	uint16_t samplehv = sampleh[0] * 256 + sampleh[1];

	//printk("sampleToV\n");
	if (0 == (ppbhv - ppbzv))
	{
	//	printk("sampleToV0\n");
		*ppb = s;
	}
	else
	{
	//	printk("sampleToV1\n");
		*ppb = (uint32_t)((float)(s - samplezv) * (ppbhv - ppbzv) / (samplehv - samplezv) + ppbzv);
	}

	*c = *ppb * 1.23;
#else

	*c = m_adc_sample;
	*ppb = m_hcho_ppb;

#endif
}

void autoUpLoadV()
{
	uint8_t ppbz[2];
	uint8_t ppbh[2];
	uint8_t samplez[2];
	uint8_t sampleh[2];
	uint8_t mode;
	uint8_t sn[5];

	readSettings(ppbz, ppbh, samplez, sampleh, &mode, sn);

	//printk("mode:%02x\n", mode);
	if (0x40 == mode)
	{
		uint16_t sample;
		uint8_t cmd[9] = {0};
		uint16_t c;
		uint16_t ppb;

		sample = m_adc_sample;
		sampleToV(sample, &c, &ppb);
		cmd[0] = 0xff;
		cmd[1] = 0x17;
		cmd[2] = 0x04;
		cmd[3] = 0x00;
		cmd[4] = ((uint8_t *) &ppb)[1];
		cmd[5] = ((uint8_t *) &ppb)[0];
		cmd[6] = 0x13;
		cmd[7] = 0x88;
		cmd[8] = commandCheckSum(cmd);
		
		for (int i = 0; i < sizeof(cmd); i ++)
		{
			uartPutChar(cmd[i]);
		}
	}
	else
	{
		
	}
}

const uint8_t commandHeadRev[] = {
	0x31,
	0x32,
	0x34,
	0x35,
	0x36,
	0x37,
	0x38,
	0x39,
	0x78,
	0x3A,
	0x86,
	0x87
};

uint8_t commandBuffer[9] = {0};
int commandNum = 0;

void doCommand()
{
	uint8_t ppbz[2];
	uint8_t ppbh[2];
	uint8_t samplez[2];
	uint8_t sampleh[2];
	uint8_t mode;
	uint8_t sn[5];

	readSettings(ppbz, ppbh, samplez, sampleh, &mode, sn);
	//printk("readSettings:%02x%02x%02x%02x%02x%02x%02x%02x\n", ppbz[0], ppbz[1], ppbh[0], ppbh[1], samplez[0], samplez[1], sampleh[0], sampleh[1]);
	//printk("command:%02x%02x%02x%02x%02x%02x%02x%02x\n", commandBuffer[0], commandBuffer[1], commandBuffer[2], commandBuffer[3], commandBuffer[4], commandBuffer[5], commandBuffer[6], commandBuffer[7]);
	commandBuffer[1] = commandBuffer[2];
	//printk("doCommand %02x\n", commandBuffer[2]);
	switch (commandBuffer[2])
	{
		case 0x31:	// 下传序列号
			sn[0] = commandBuffer[3];
			sn[1] = commandBuffer[4];
			sn[2] = commandBuffer[5];
			sn[3] = commandBuffer[6];
			sn[4] = commandBuffer[7];
			writeSettings(ppbz, ppbh, samplez, sampleh, mode, sn);
			commandBuffer[2] = sn[0];
			commandBuffer[3] = sn[1];
			commandBuffer[4] = sn[2];
			commandBuffer[5] = sn[3];
			commandBuffer[6] = sn[4];
			commandBuffer[7] = 0;
			break;
		case 0x32:	// 读取序列号
			commandBuffer[2] = sn[0];
			commandBuffer[3] = sn[1];
			commandBuffer[4] = sn[2];
			commandBuffer[5] = sn[3];
			commandBuffer[6] = sn[4];
			commandBuffer[7] = 0;
			break;
		case 0x34:	// 读校准浓度值
			commandBuffer[2] = ppbz[0];
			commandBuffer[3] = ppbz[1];
			commandBuffer[4] = ppbh[0];
			commandBuffer[5] = ppbh[1];
			commandBuffer[6] = 0;
			commandBuffer[7] = 0;
			break;
		case 0x35:	// 读校准采样值
			commandBuffer[2] = samplez[0];
			commandBuffer[3] = samplez[1];
			commandBuffer[4] = sampleh[0];
			commandBuffer[5] = sampleh[1];
			commandBuffer[6] = 0;
			commandBuffer[7] = 0;
			break;
		case 0x36:	// 甲醇零点校准
			break;
		case 0x37:	// 甲醇高点校准
			break;
		case 0x38:	// 甲醛零点校准
			ppbz[0] = commandBuffer[3];
			ppbz[1] = commandBuffer[4];
			samplez[0] = commandBuffer[5];
			samplez[1] = commandBuffer[6];
			//printk("command0x38:%02x%02x%02x%02x%02x%02x%02x%02x\n", ppbz[0], ppbz[1], ppbh[0], ppbh[1], samplez[0], samplez[1], sampleh[0], sampleh[1]);
			writeSettings(ppbz, ppbh, samplez, sampleh, mode, sn);
			commandBuffer[2] = ppbz[0];
			commandBuffer[3] = ppbz[1];
			commandBuffer[4] = samplez[0];
			commandBuffer[5] = samplez[1];
			commandBuffer[6] = 0;
			commandBuffer[7] = 0;
			break;
		case 0x39:	// 混气校准
			ppbh[0] = commandBuffer[3];
			ppbh[1] = commandBuffer[4];
			sampleh[0] = commandBuffer[5];
			sampleh[1] = commandBuffer[6];
			writeSettings(ppbz, ppbh, samplez, sampleh, mode, sn);
			commandBuffer[2] = ppbh[0];
			commandBuffer[3] = ppbh[1];
			commandBuffer[4] = sampleh[0];
			commandBuffer[5] = sampleh[1];
			commandBuffer[6] = 0;
			commandBuffer[7] = 0;
			break;
		case 0x78:
		case 0x3A:	// 通信切换命令
			mode = commandBuffer[3];
			writeSettings(ppbz, ppbh, samplez, sampleh, mode, sn);
			commandBuffer[2] = mode;
			commandBuffer[3] = 0;
			commandBuffer[4] = 0;
			commandBuffer[5] = 0;
			commandBuffer[6] = 0;
			commandBuffer[7] = 0;
			break;
		case 0x86:	// 读气体浓度
			{
				uint16_t ppbv;
#if 0
				uint16_t ppbzv = ppbz[0] * 256 + ppbz[1];
				uint16_t ppbhv = ppbh[0] * 256 + ppbh[1];
				uint16_t samplezv = samplez[0] * 256 + samplez[1];
				uint16_t samplehv = sampleh[0] * 256 + sampleh[1];

				if (0xffff == samplezv)
				{
					ppbv = 0;
				}
				else
				{
					ppbv = (uint32_t)(m_sample_buffer[0] - ppbzv) * (samplehv - samplezv) / (ppbhv - ppbzv);
				}
#else
				uint16_t c;
				sampleToV(m_adc_sample, &c, &ppbv);
#endif
				commandBuffer[2] = ((uint8_t*) &c)[1];
				commandBuffer[3] = ((uint8_t*) &c)[0];
				commandBuffer[4] = 0;
				commandBuffer[5] = 0;
				commandBuffer[6] = ((uint8_t*) &ppbv)[1];
				commandBuffer[7] = ((uint8_t*) &ppbv)[0];
			}
			break;
		case 0x87:	// 读气体采样值
			commandBuffer[2] = ((uint8_t*) &(m_adc_sample))[1];
			commandBuffer[3] = ((uint8_t*) &(m_adc_sample))[0];
			commandBuffer[4] = 0;
			commandBuffer[5] = 0;
			commandBuffer[6] = 0;
			commandBuffer[7] = 0;
			break;
		default:
			return;
	}

	{
		uint8_t checksum = commandCheckSum(commandBuffer);
		
		commandBuffer[8] = checksum;
	}
	{
		for (int i = 0; i < sizeof(commandBuffer); i ++)
		{
			uartPutChar(commandBuffer[i]);
		}
	}
}

void formaldehyde_deal(uint8_t chardata)
{
	//printk("%c", chardata);
	if (0 == commandNum)
	{
		if (0xff == chardata)
		{
			commandBuffer[0] = 0xff;
			commandNum = 1;
			//printk("0");
		}
	}
	else if (1 == commandNum)
	{
		if (0x01 == chardata)
		{
			commandBuffer[1] = 0x01;
			commandNum = 2;
			//printk("1");
		}
		else
		{
			commandNum = 0;
		}
	}
	else if (2 == commandNum)
	{
		for (int i = 0; i < sizeof(commandHeadRev); i ++)
		{
			if (chardata == commandHeadRev[i])
			{
				commandBuffer[2] = chardata;
				commandNum = 3;
				//printk("3");
				return;
			}
		}
		commandNum = 0;
	}
	else
	{
		if ((0xff == commandBuffer[commandNum - 2]) && (0x01 == commandBuffer[commandNum - 1]))
		{
			for (int i = 0; i < sizeof(commandHeadRev); i ++)
			{
				if (chardata == commandHeadRev[i])
				{
					commandBuffer[2] = chardata;
					commandNum = 3;
					return;
				}
			}
		}
		commandBuffer[commandNum ++] = chardata;

		if (9 <= commandNum)
		{
			uint8_t checksum = commandCheckSum(commandBuffer);
			if (checksum == commandBuffer[8])
			{
			//printk("09");
				// do command
				doCommand();
				commandNum = 0;
			}
			else
			{
				if (0xff == commandBuffer[8])
				{
					commandBuffer[0] = 0xff;
					commandNum = 1;
				}
				else if (0xff == commandBuffer[7] && 0x01 == commandBuffer[8])
				{
					commandBuffer[0] = 0xff;
					commandBuffer[1] = 0x01;
					commandNum = 2;
				}
				else
				{
					commandNum = 0;
				}
			}
		}
	}
}

static void uart_fifo_callback(const struct device *dev, void *user_data)
{
	uint8_t recvData;
	static int tx_data_idx;

	ARG_UNUSED(user_data);

	/* Verify uart_irq_update() */
	if (!uart_irq_update(dev)) {
		//TC_PRINT("retval should always be 1\n");
		printk("retval should always be 1\n");
		return;
	}

	/* Verify uart_irq_rx_ready() */
	if (uart_irq_rx_ready(dev)) {
		/* Verify uart_fifo_read() */
		uart_fifo_read(dev, &recvData, 1);

		// printk("\n%c\n", recvData);
		// z_impl_uart_poll_out(dev, recvData + 26);
		formaldehyde_deal(recvData);

		if ((recvData == '\n') || (recvData == '\r')) {
			data_received = true;
		}
	}
}

static void test_fifo_read(void)
{
	uart_dev = device_get_binding(UART_DEVICE_NAME);

	/* Verify uart_irq_callback_set() */
	uart_irq_callback_set(uart_dev, uart_fifo_callback);

	/* Enable Tx/Rx interrupt before using fifo */
	/* Verify uart_irq_rx_enable() */
	uart_irq_rx_enable(uart_dev);

}

#endif



//////////////////////// ble

uint16_t but_val;

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
	printk("recv:");
	for (size_t i = 0; i < len; i++)
	{
		/* code */
		printk(" %02x ", ((char *)buf)[i]);
	}
	if (len == 14)
	{
		// settings
		setWriteSettingsBuffer(buf);
	}
	else if (len == 4)
	{
		// adv + val
		m_adc_sample = ((int16_t *) buf)[0];
		m_hcho_ppb = ((int16_t *) buf)[1];
	}

	printk("\n");

	return 0;
}

static ssize_t read_tx(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     void *buf, uint16_t len, uint16_t offset)
{
	char buff[14] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

	getWriteSettingsBuffer(buff);
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

//////////////////////// ble


void main(void)
{
	printk("Bluetooth main\n");
	
#if 1
	int err;

	bt_conn_cb_register(&conn_callbacks);

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(bt_ready);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
	}

	{
		k_thread_create(&settingsUpdate_thread_data, settingsUpdate_thread_stack,
			K_THREAD_STACK_SIZEOF(settingsUpdate_thread_stack),
			modulate_settingsUpdate, NULL, NULL, NULL,
			K_PRIO_COOP(10),
			0, K_NO_WAIT);
		k_thread_name_set(&settingsUpdate_thread_data, "flash thread");
	}

#endif


#if 1
	test_fifo_read();
	//init_rx_queue();


	/* Implement notification. At the moment there is no suitable way
	 * of starting delayed work so we do it here
	 */
	while (1) {
		k_sleep(K_SECONDS(1));

	}
#endif
}
