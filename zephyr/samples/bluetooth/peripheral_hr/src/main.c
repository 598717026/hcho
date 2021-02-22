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

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/services/bas.h>
#include <bluetooth/services/hrs.h>


#define	ZERO_ADC_V		184

void sampleToV(uint16_t s, uint16_t *c, uint16_t *ppb);
void autoUpLoadV();

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

#define BUFFER_SIZE  6
static  int16_t m_sample_buffer[BUFFER_SIZE];
static  int16_t m_adc_sample;

struct bt_conn *default_conn;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(
		BT_DATA_UUID16_ALL,
	//	      BT_UUID_16_ENCODE(BT_UUID_HRS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),
	//	      BT_UUID_16_ENCODE(BT_UUID_DIS_VAL)
			  )
};


static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		default_conn = bt_conn_ref(conn);
		printk("Connected\n");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason 0x%02x)\n", reason);

	if (default_conn) {
		bt_conn_unref(default_conn);
		default_conn = NULL;
	}
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(void)
{
	int err;

	printk("Bluetooth initialized\n");

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.cancel = auth_cancel,
};

#if 1

static void bas_notify(void)
{
	uint16_t battery_level = bt_bas_get_battery_level();

	battery_level--;

	if (!battery_level) {
		battery_level = 1000U;
	}

//	printk("bas_notify %d %08x\n", battery_level, battery_level);
//	bt_bas_set_battery_level(battery_level);
	
//	printk("bas_notify %d %08x\n", m_adc_sample, m_adc_sample);
	uint16_t c;
	uint16_t ppb;
	sampleToV(m_adc_sample, &c, &ppb);
	bt_bas_set_battery_level(ppb);
}

#endif

#if 0
static void hrs_notify(void)
{
	static uint8_t heartrate = 90U;

	/* Heartrate measurements simulation */
	heartrate++;
	if (heartrate == 160U) {
		heartrate = 90U;
	}

	printk("hrs_notify %d %08x\n", heartrate, heartrate);
	bt_hrs_notify(heartrate);
}

#endif


#include <drivers/adc.h>
#include <hal/nrf_saadc.h>
#define ADC_DEVICE_NAME		DT_LABEL(DT_INST(0, nordic_nrf_saadc))
#define ADC_RESOLUTION		12
#define ADC_GAIN		ADC_GAIN_1_6
#define ADC_REFERENCE		ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME	ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10)
#define ADC_1ST_CHANNEL_ID	0
#define ADC_1ST_CHANNEL_INPUT	NRF_SAADC_INPUT_AIN0
#define ADC_2ND_CHANNEL_ID	2
#define ADC_2ND_CHANNEL_INPUT	NRF_SAADC_INPUT_AIN2

/* Invalid value that is not supposed to be written by the driver. It is used
 * to mark the sample buffer entries as empty. If needed, it can be overriden
 * for a particular board by providing a specific definition above.
 */
#if !defined(INVALID_ADC_VALUE)
#define INVALID_ADC_VALUE SHRT_MIN
#endif

static const struct adc_channel_cfg m_1st_channel_cfg = {
	.gain             = ADC_GAIN,
	.reference        = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id       = ADC_1ST_CHANNEL_ID,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
	.input_positive   = ADC_1ST_CHANNEL_INPUT,
#endif
};
const struct device *get_adc_device(void)
{
	return device_get_binding(ADC_DEVICE_NAME);
}

static const struct device *init_adc(void)
{
	int i, ret;
	const struct device *adc_dev = device_get_binding(ADC_DEVICE_NAME);

	if (NULL == adc_dev)
	{
		printk("Cannot get ADC device\n");
		return NULL;
	}
	ret = adc_channel_setup(adc_dev, &m_1st_channel_cfg);
	if (ret != 0)
	{
		printk("Setting up of the first channel failed with code %d\n", ret);
		return NULL;
	}

	for (i = 0; i < BUFFER_SIZE; ++i) {
		m_sample_buffer[i] = INVALID_ADC_VALUE;
	}

	printk("Get ADC device OK\n");
	return adc_dev;
}

/*
 * test_adc_sample_one_channel
 */
static int test_task_one_channel(void)
{
	int ret;
	const struct adc_sequence sequence = {
		.channels    = BIT(ADC_1ST_CHANNEL_ID),
		.buffer      = m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer),
		.resolution  = ADC_RESOLUTION,
	};

	const struct device *adc_dev = init_adc();

	if (!adc_dev) {
		return -1;
	}

	ret = adc_read(adc_dev, &sequence);
	//assert_equal(ret, 0, "adc_read() failed with code %d", ret);

	//check_samples(1);

	return 0;
}

const struct adc_sequence sequence = {
	.channels    = BIT(ADC_1ST_CHANNEL_ID),
	.buffer      = m_sample_buffer,
	.buffer_size = sizeof(m_sample_buffer),
	.resolution  = ADC_RESOLUTION,
};

struct device *adc_dev;
static struct k_thread adc_thread_data;
static K_THREAD_STACK_DEFINE(adc_thread_stack, 320);


// 递推平均滤波法（滑动平均滤波法）
/*
 * description: 把连续取N个采样值看成一个队列，队列的长度固定为N，
 *              每次采样到一个新数据放入队尾，并扔掉原来队首的一次数据（先入先出原则），
 *              把队列中的N个数据进行算术平均运算，就可获得新的滤波结果
 *              N值的选取：流量，N=12；压力：N=4；液面，N=4~12；温度：N=1~4
 * advantage: 对周期性干扰有良好的抑制作用，平滑度高，适用于高频震荡的系统
 * disadvantage: 灵敏度低，对偶然出现的脉冲性干扰的抑制作用较差，
 *               不易消除由于脉冲干扰所引起的采样值偏差，
 *               不适用于脉冲干扰比较严重的场合，比较浪费ram
*/
#define	N	96

uint16_t adc_filter(uint16_t s)
{
    int count;
    uint32_t sum = 0;
	uint16_t value_sbuf[N] = {0};
	static uint16_t value_buf[N] = {0};
	static uint16_t i = 0;
 
//	printk("adc_filter %08x\n", s);
	s += ZERO_ADC_V;
	if (s & 0x8000)
	{
		s = 0;
	}
    value_buf[i++] = s;
	//printk("ADC %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", value_buf[0], value_buf[1], value_buf[2], value_buf[3], value_buf[4], value_buf[5], value_buf[6], value_buf[7], value_buf[8], value_buf[9], value_buf[10], value_buf[11]);
    if ( i >= N )
    {
        i = 0;
    }

#if 0

    for (count = 0; count < N; count++)
    {
        value_sbuf[count] = value_buf[count];
    }
	
	for(int i = 0; i < N - 1; i++)
	{
		for(int j = i + 1; j < N; j++)
		{
			if(value_sbuf[i] > value_sbuf[j])
			{
				int temp = value_sbuf[i];
				value_sbuf[i] = value_sbuf[j];
				value_sbuf[j] = temp;
			}
		}
	}
	//printk("ADC %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", value_sbuf[0], value_sbuf[1], value_sbuf[2], value_sbuf[3], value_sbuf[4], value_sbuf[5], value_sbuf[6], value_sbuf[7], value_sbuf[8], value_sbuf[9], value_sbuf[10], value_sbuf[11]);

    for (count = N / 8; count < N - N / 8; count++)
    {
        sum += value_sbuf[count];
    }
    return (uint16_t)(sum / (N - N / 8 - N / 8));

#else

    for (count = 0; count < N; count++)
    {
        sum += value_buf[count];
    }
    return (uint16_t)(sum / N);

#endif
}

void modulate_adc(void *p1, void *p2, void *p3)
{
	int8_t txp_get = 0;
	uint8_t idx = 0;
	int ret;
	while (1) {
		ret = adc_read(adc_dev, &sequence);
		if (ret != 0)
		{
			printk("adc_read() failed with code %d", ret);
		}
		else {
			int i;
			for (i = 0;i < BUFFER_SIZE;i++)
			{
				//printk("adc buffer[%d] = %d\n", i, m_sample_buffer[i]);

			}
			// m_adc_sample = m_sample_buffer[0];
			m_adc_sample = adc_filter(m_sample_buffer[0]);
			//printk("adc %d, %d\n", m_adc_sample, m_sample_buffer[0]);
		}
		k_sleep(K_MSEC(200));
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

#if 0

#include <drivers/uart.h>
/* RX queue */
static struct k_fifo rx_queue;
static K_THREAD_STACK_DEFINE(rx_stack, 1024);
static struct k_thread rx_thread_data;

static void rx_thread(void)
{
	printk("RX thread started");

#if 0

	while (true) {
		//struct net_pkt *pkt;
		//struct net_buf *buf;
		uint8_t *buf;
		uint8_t specifier;

		//pkt = k_fifo_get(&rx_queue, K_FOREVER);
		//buf = net_buf_frag_last(pkt->buffer);

		buf = k_fifo_get(&rx_queue, K_FOREVER);

		//LOG_DBG("rx_queue pkt %p buf %p", pkt, buf);

		LOG_HEXDUMP_DBG(buf->data, buf->len, "SLIP >");

		/* TODO: process */
		specifier = net_buf_pull_u8(buf);
		switch (specifier) {
		case '?':
			process_request(buf);
			break;
		case '!':
			process_config(pkt);
			break;
		default:
			LOG_ERR("Unknown message specifier %c", specifier);
			break;
		}

		net_pkt_unref(pkt);
	}

#endif

	const struct device *uart_dev = device_get_binding(CONFIG_UART_CONSOLE_ON_DEV_NAME);

	uint8_t recvc;
	int recvnum = 0;
	uint8_t recvData[128];
	int rev;
	while(true)
	{
		k_sleep(K_MSEC(10));
		recvnum = 0;
		while (0 == z_impl_uart_poll_in(uart_dev, &recvc))
		{
			recvData[recvnum ++] = recvc;
		}
		
		while (recvnum --)
		{
			printk("rx_thread %c\n", recvData[recvnum]);
		}
	}
}

static void init_rx_queue(void)
{
	k_fifo_init(&rx_queue);

	k_thread_create(&rx_thread_data, rx_stack,
			K_THREAD_STACK_SIZEOF(rx_stack),
			(k_thread_entry_t)rx_thread,
			NULL, NULL, NULL, K_PRIO_COOP(8), 0, K_NO_WAIT);
}

#endif



#include <drivers/i2c.h>

//#define I2C_DEV DT_LABEL(DT_ALIAS(i2c_0))

#define I2C_DEV "I2C_0"

/**
 * @file Sample app using the Fujitsu MB85RC256V FRAM through ARC I2C.
 */

#define FRAM_I2C_ADDR	0x50

static int write_bytes(const struct device *i2c_dev, uint16_t addr,
		       uint8_t *data, uint32_t num_bytes)
{
	uint8_t wr_addr[2];
	struct i2c_msg msgs[2];

	/* FRAM address */
	wr_addr[0] = (addr >> 8) & 0xFF;
	wr_addr[1] = addr & 0xFF;

	/* Setup I2C messages */

	/* Send the address to write to */
	msgs[0].buf = wr_addr;
	msgs[0].len = 2U;
	msgs[0].flags = I2C_MSG_WRITE;

	/* Data to be written, and STOP after this. */
	msgs[1].buf = data;
	msgs[1].len = num_bytes;
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer(i2c_dev, &msgs[0], 2, FRAM_I2C_ADDR);
}

static int read_bytes(const struct device *i2c_dev, uint16_t addr,
		      uint8_t *data, uint32_t num_bytes)
{
	uint8_t wr_addr[2];
	struct i2c_msg msgs[2];

	/* Now try to read back from FRAM */

	/* FRAM address */
	wr_addr[0] = (addr >> 8) & 0xFF;
	wr_addr[1] = addr & 0xFF;

	/* Setup I2C messages */

	/* Send the address to read from */
	msgs[0].buf = wr_addr;
	msgs[0].len = 2U;
	msgs[0].flags = I2C_MSG_WRITE;

	/* Read from device. STOP after this. */
	msgs[1].buf = data;
	msgs[1].len = num_bytes;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	return i2c_transfer(i2c_dev, &msgs[0], 2, FRAM_I2C_ADDR);
}


void i2c_main(void)
{
	const struct device *i2c_dev;
	uint8_t cmp_data[16];
	uint8_t data[16];
	int i, ret;

	i2c_dev = device_get_binding(I2C_DEV);
	if (!i2c_dev) {
		printk("I2C: Device driver not found.\n");
		return;
	}

	/* Do one-byte read/write */
	data[0] = 0xAE;
	ret = write_bytes(i2c_dev, 0x00, &data[0], 1);
	if (ret) {
		printk("Error writing to FRAM! error code (%d)\n", ret);
		return;
	} else {
		printk("Wrote 0xAE to address 0x00.\n");
	}
	

	data[0] = 0x86;
	ret = write_bytes(i2c_dev, 0x01, &data[0], 1);
	if (ret) {
		printk("Error writing to FRAM! error code (%d)\n", ret);
		return;
	} else {
		printk("Wrote 0x86 to address 0x01.\n");
	}

	data[0] = 0x00;
	ret = read_bytes(i2c_dev, 0x00, &data[0], 1);
	if (ret) {
		printk("Error reading from FRAM! error code (%d)\n", ret);
		return;
	} else {
		printk("Read 0x%X from address 0x00.\n", data[0]);
	}

	data[1] = 0x00;
	ret = read_bytes(i2c_dev, 0x01, &data[0], 1);
	if (ret) {
		printk("Error reading from FRAM! error code (%d)\n", ret);
		return;
	} else {
		printk("Read 0x%X from address 0x01.\n", data[0]);
	}

	/* Do multi-byte read/write */

	/* get some random data, and clear out data[] */
	for (i = 0; i < sizeof(cmp_data); i++) {
		cmp_data[i] = k_cycle_get_32() & 0xFF;
		data[i] = 0x00;
	}

	/* write them to the FRAM */
	ret = write_bytes(i2c_dev, 0x00, cmp_data, sizeof(cmp_data));
	if (ret) {
		printk("Error writing to FRAM! error code (%d)\n", ret);
		return;
	} else {
		printk("Wrote %zu bytes to address 0x00.\n", sizeof(cmp_data));
	}

	ret = read_bytes(i2c_dev, 0x00, data, sizeof(data));
	if (ret) {
		printk("Error reading from FRAM! error code (%d)\n", ret);
		return;
	} else {
		printk("Read %zu bytes from address 0x00.\n", sizeof(data));
	}

	ret = 0;
	for (i = 0; i < sizeof(cmp_data); i++) {
		/* uncomment below if you want to see all the bytes */
		/* printk("0x%X ?= 0x%X\n", cmp_data[i], data[i]); */
		if (cmp_data[i] != data[i]) {
			printk("Data comparison failed @ %d.\n", i);
			ret = -EIO;
		}
	}
	if (ret == 0) {
		printk("Data comparison successful.\n");
	}
}




void main(void)
{
	printk("Bluetooth main\n");
	
#if 1
	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	bt_ready();

	bt_conn_cb_register(&conn_callbacks);
	bt_conn_auth_cb_register(&auth_cb_display);

	{
		k_thread_create(&settingsUpdate_thread_data, settingsUpdate_thread_stack,
			K_THREAD_STACK_SIZEOF(settingsUpdate_thread_stack),
			modulate_settingsUpdate, NULL, NULL, NULL,
			K_PRIO_COOP(10),
			0, K_NO_WAIT);
		k_thread_name_set(&settingsUpdate_thread_data, "flash thread");
	}

	int ret;
	adc_dev = init_adc();

	if (adc_dev) {
		k_thread_create(&adc_thread_data, adc_thread_stack,
			K_THREAD_STACK_SIZEOF(adc_thread_stack),
			modulate_adc, NULL, NULL, NULL,
			K_PRIO_COOP(10),
			0, K_NO_WAIT);
		k_thread_name_set(&adc_thread_data, "adc thread");
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

		/* Heartrate measurements simulation */
	//	hrs_notify();

		/* Battery level simulation */
		bas_notify();

		
	}
#endif
}
