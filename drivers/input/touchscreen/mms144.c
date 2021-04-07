/*
 * mms144.c - Touchscreen driver for Melfas MMS144 touch controllers
 *
 * Copyright (C) 2011 Google Inc.
 * Author: Dima Zavin <dima@xxxxxxxxxxx>
 * Simon Wilson <simonwilson@xxxxxxxxxx>
 * Copyright (C) 2012 Nikolay Epifanov <nik.epifanov@xxxxxxxxx>
 * Copyright (C) 2021 Marco Scardovi <marco@xxxxxxxxx>
 *
 * ISP reflashing code based on original code from Melfas.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c/mms144.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/unaligned/le_struct.h>

#define MAX_FINGERS 10
#define MAX_WIDTH 30
#define MAX_PRESSURE 255
#define FINGER_EVENT_SZ 6

/* Registers */
#define MMS_MODE_CONTROL 0x01
#define MMS_XYRES_HI 0x02
#define MMS_XRES_LO 0x03
#define MMS_YRES_LO 0x04

#define MMS_INPUT_EVENT_PKT_SZ 0x0F
#define MMS_INPUT_EVENT0 0x10

#define MMS_TSP_REVISION 0xF0
#define MMS_HW_REVISION 0xF1
#define MMS_COMPAT_GROUP 0xF2
#define MMS_FW_VERSION 0xF3

#define ISP_IC_INFO_ADDR 0x1F00
/* each address addresses 4-byte words */
#define ISP_MAX_FW_SIZE (ISP_IC_INFO_ADDR * 4)

#define FW_HEADER_VERSION 1

enum {
	ISP_MODE_FLASH_ERASE = 0x59F3,
	ISP_MODE_FLASH_WRITE = 0x62CD,
	ISP_MODE_FLASH_READ = 0x6AC9,
};

static bool mms_force_reflash = false;
module_param_named(force_reflash, mms_force_reflash, bool, S_IWUSR | S_IRUGO);

struct mms_ts_info {
	struct i2c_client *client;
	struct input_dev *input_dev;
	char phys[32];
	int irq;
	struct mms144_platform_data pdata;

	/* protects the enabled flag */
	struct mutex lock;
	bool enabled;
};

struct mms_fw_image {
	__le32 hdr_len;
	__le32 data_len;
	__le32 fw_ver;
	__le32 hdr_ver;
	u8 data[0];
} __packed;

static irqreturn_t mms_ts_interrupt(int irq, void *dev_id) {
	struct mms_ts_info *info = dev_id;
	struct i2c_client *client = info->client;
	u8 buf[MAX_FINGERS * FINGER_EVENT_SZ] = { 0 };
	int ret, i, sz, x, y, id, pressed, width, strength;
	u8 reg = MMS_INPUT_EVENT0;
	u8 *tmp;
	struct i2c_msg msg[] = { {
			.addr = client->addr,
			.flags = 0,
			.buf = &reg,
			.len = 1,
		}, {
		.addr = client->addr,
		.flags = I2C_M_RD,
		.buf = buf,
		},
	};

	sz = i2c_smbus_read_byte_data(client, MMS_INPUT_EVENT_PKT_SZ);
	if (sz < 0) {
		dev_err(&client->dev, "%s bytes=%d\n", __func__, sz);
		goto out;
	}
	dev_dbg(&client->dev, "bytes available: %d\n", sz);
	BUG_ON(sz > MAX_FINGERS * FINGER_EVENT_SZ);
	if (sz == 0) goto out;

	msg[1].len = sz;
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret != ARRAY_SIZE(msg)) {
		dev_err(&client->dev, "failed to read %d bytes of touch data (%d)\n", sz, ret);
		goto out;
	}

	#if defined(VERBOSE_DEBUG)
		print_hex_dump(KERN_DEBUG, "mms144 raw: ",
		DUMP_PREFIX_OFFSET, 32, 1, buf, sz, false);
	#endif
	for (i = 0; i < sz; i += FINGER_EVENT_SZ) {
		tmp = &buf[i];
		id = (tmp[0] & 0xf) - 1;
		pressed = tmp[0] & 0x80;
		x = tmp[2] | ((tmp[1] & 0xf) << 8);
		y = tmp[3] | ((tmp[1] >> 4) << 8);
		width = tmp[4];
		strength = tmp[5];

		x = min(x, info->pdata.max_x);
		y = min(y, info->pdata.max_y);
		if (info->pdata.invert_x)
		x = info->pdata.max_x - x;
		if (info->pdata.invert_y)
		y = info->pdata.max_y - y;

		input_mt_slot(info->input_dev, id);
		input_mt_report_slot_state(info->input_dev,
		MT_TOOL_FINGER, pressed);
		if (!pressed) {
			dev_dbg(&client->dev, "finger %d up\n", id);
			continue;
		}

		input_report_abs(info->input_dev, ABS_MT_TOUCH_MAJOR, width);
		input_report_abs(info->input_dev, ABS_MT_PRESSURE, strength);
		input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);
		dev_dbg(&client->dev, "id: %d, pressed: %d, x: %d, y: %d, "
		"width: %d, strength: %d\n",
		id, pressed, x, y,
		width, strength);
	}
	input_mt_report_pointer_emulation(info->input_dev, false);
	input_sync(info->input_dev);

	out:
	return IRQ_HANDLED;
}

static void hw_reboot(struct mms_ts_info *info, bool bootloader) {
	gpio_direction_output(info->pdata.gpio_vdd_en, 0);
	gpio_direction_output(info->pdata.gpio_sda, bootloader ? 0 : 1);
	gpio_direction_output(info->pdata.gpio_scl, bootloader ? 0 : 1);
	gpio_direction_output(info->pdata.gpio_resetb, 0);
	msleep(30);
	gpio_set_value(info->pdata.gpio_vdd_en, 1);
	msleep(30);

	if (bootloader) {
		gpio_set_value(info->pdata.gpio_scl, 0);
		gpio_set_value(info->pdata.gpio_sda, 1);
	} else {
		gpio_set_value(info->pdata.gpio_resetb, 1);
		gpio_direction_input(info->pdata.gpio_resetb);
		gpio_direction_input(info->pdata.gpio_scl);
		gpio_direction_input(info->pdata.gpio_sda);
	}
	msleep(40);
}

static inline void hw_reboot_bootloader(struct mms_ts_info *info) {
	hw_reboot(info, true);
}

static inline void hw_reboot_normal(struct mms_ts_info *info) {
	hw_reboot(info, false);
}

static void mms_pwr_on_reset(struct mms_ts_info *info) {
	struct i2c_adapter *adapter = to_i2c_adapter(info->client->dev.parent);

	i2c_lock_adapter(adapter);
	info->pdata.mux_fw_flash(true);

	gpio_direction_output(info->pdata.gpio_vdd_en, 0);
	gpio_direction_output(info->pdata.gpio_sda, 1);
	gpio_direction_output(info->pdata.gpio_scl, 1);
	gpio_direction_output(info->pdata.gpio_resetb, 1);
	msleep(50);
	gpio_direction_output(info->pdata.gpio_vdd_en, 1);
	msleep(50);

	info->pdata.mux_fw_flash(false);
	i2c_unlock_adapter(adapter);

	/* TODO: Seems long enough for the firmware to boot.
	* Find the right value */
 msleep(250);
}

static void isp_toggle_clk(struct mms_ts_info *info, int start_lvl, int end_lvl, int hold_us) {
	gpio_set_value(info->pdata.gpio_scl, start_lvl);
	udelay(hold_us);
	gpio_set_value(info->pdata.gpio_scl, end_lvl);
	udelay(hold_us);
}

/* 1 <= cnt <= 32 bits to write */
static void isp_send_bits(struct mms_ts_info *info, u32 data, int cnt) {
	gpio_direction_output(info->pdata.gpio_resetb, 0);
	gpio_direction_output(info->pdata.gpio_scl, 0);
	gpio_direction_output(info->pdata.gpio_sda, 0);

	/* clock out the bits, msb first */
	while (cnt--) {
	gpio_set_value(info->pdata.gpio_sda, (data >> cnt) & 1);
	udelay(3);
	isp_toggle_clk(info, 1, 0, 3);
	}
}

/* 1 <= cnt <= 32 bits to read */
static u32 isp_recv_bits(struct mms_ts_info *info, int cnt) {
	u32 data = 0;

	gpio_direction_output(info->pdata.gpio_resetb, 0);
	gpio_direction_output(info->pdata.gpio_scl, 0);
	gpio_set_value(info->pdata.gpio_sda, 0);
	gpio_direction_input(info->pdata.gpio_sda);

	/* clock in the bits, msb first */
	while (cnt--) {
		isp_toggle_clk(info, 0, 1, 1);
		data = (data << 1) | (!!gpio_get_value(info->pdata.gpio_sda));
	}

	gpio_direction_output(info->pdata.gpio_sda, 0);
	return data;
}

static void isp_enter_mode(struct mms_ts_info *info, u32 mode) {
	int cnt;
	unsigned long flags;

	local_irq_save(flags);
	gpio_direction_output(info->pdata.gpio_resetb, 0);
	gpio_direction_output(info->pdata.gpio_scl, 0);
	gpio_direction_output(info->pdata.gpio_sda, 1);

	mode &= 0xffff;
	for (cnt = 15; cnt >= 0; cnt--) {
		gpio_set_value(info->pdata.gpio_resetb, (mode >> cnt) & 1);
		udelay(3);
		isp_toggle_clk(info, 1, 0, 3);
	}

	gpio_set_value(info->pdata.gpio_resetb, 0);
	local_irq_restore(flags);
}

static void isp_exit_mode(struct mms_ts_info *info)
{
	int i;
	unsigned long flags;

	local_irq_save(flags);
	gpio_direction_output(info->pdata.gpio_resetb, 0);
	udelay(3);

	for (i = 0; i < 10; i++)
	isp_toggle_clk(info, 1, 0, 3);
	local_irq_restore(flags);
}

static void flash_set_address(struct mms_ts_info *info, u16 addr) {
	/* Only 13 bits of addr are valid.
	* The addr is in bits 13:1 of cmd */
	isp_send_bits(info, (u32)(addr & 0x1fff) << 1, 18);
}

static void flash_erase(struct mms_ts_info *info) {
	isp_enter_mode(info, ISP_MODE_FLASH_ERASE);

	gpio_direction_output(info->pdata.gpio_resetb, 0);
	gpio_direction_output(info->pdata.gpio_scl, 0);
	gpio_direction_output(info->pdata.gpio_sda, 1);

	/* 4 clock cycles with different timings for the erase to get processed, clk is already 0 from above */
	udelay(7);
	isp_toggle_clk(info, 1, 0, 3);
	udelay(7);
	isp_toggle_clk(info, 1, 0, 3);
	usleep_range(25000, 35000);
	isp_toggle_clk(info, 1, 0, 3);
	usleep_range(150, 200);
	isp_toggle_clk(info, 1, 0, 3);

	gpio_set_value(info->pdata.gpio_sda, 0);

	isp_exit_mode(info);
}

static u32 flash_readl(struct mms_ts_info *info, u16 addr) {
	int i;
	u32 val;
	unsigned long flags;

	local_irq_save(flags);
	isp_enter_mode(info, ISP_MODE_FLASH_READ);
	flash_set_address(info, addr);

	gpio_direction_output(info->pdata.gpio_scl, 0);
	gpio_direction_output(info->pdata.gpio_sda, 0);
	udelay(40);

	/* data load cycle */
	for (i = 0; i < 6; i++)
	isp_toggle_clk(info, 1, 0, 10);

	val = isp_recv_bits(info, 32);
	isp_exit_mode(info);
	local_irq_restore(flags);

	return val;
}

static void flash_writel(struct mms_ts_info *info, u16 addr, u32 val) {
	unsigned long flags;

	local_irq_save(flags);
	isp_enter_mode(info, ISP_MODE_FLASH_WRITE);
	flash_set_address(info, addr);
	isp_send_bits(info, val, 32);

	gpio_direction_output(info->pdata.gpio_sda, 1);
	/* 6 clock cycles with different timings for the data to get written
	* into flash */
	isp_toggle_clk(info, 0, 1, 3);
	isp_toggle_clk(info, 0, 1, 3);
	isp_toggle_clk(info, 0, 1, 6);
	isp_toggle_clk(info, 0, 1, 12);
	isp_toggle_clk(info, 0, 1, 3);
	isp_toggle_clk(info, 0, 1, 3);

	isp_toggle_clk(info, 1, 0, 1);

	gpio_direction_output(info->pdata.gpio_sda, 0);
	isp_exit_mode(info);
	local_irq_restore(flags);
	usleep_range(300, 400);
}

static bool flash_is_erased(struct mms_ts_info *info) {
	struct i2c_client *client = info->client;
	u32 val;
	u16 addr;

	for (addr = 0; addr < (ISP_MAX_FW_SIZE / 4); addr++) {
		udelay(40);
		val = flash_readl(info, addr);

		if (val != 0xffffffff) {
			dev_dbg(&client->dev,
			"addr 0x%x not erased: 0x%08x != 0xffffffff\n",
			addr, val);
			return false;
		}
	}
	return true;
}

static int fw_write_image(struct mms_ts_info *info, const u8 *data, size_t len) {
	struct i2c_client *client = info->client;
	u16 addr;
	u32 val, verify_val;
	int retries;

	for (addr = 0; addr < (len / 4); addr++, data += 4) {
		val = get_unaligned_le32(data);
		retries = 3;

		while (retries--) {
			flash_writel(info, addr, val);
			verify_val = flash_readl(info, addr);
			if (val == verify_val)
			break;
			dev_err(&client->dev,
			"mismatch @ addr 0x%x: 0x%x != 0x%x\n",
			addr, verify_val, val);
			hw_reboot_bootloader(info);
			continue;
		}
			if (retries < 0)
			return -ENXIO;
	}
	return 0;
}

static int fw_download(struct mms_ts_info *info, const u8 *data, size_t len) {
	struct i2c_client *client = info->client;
	u32 val;
	int ret = 0;

	if (len % 4) {
		dev_err(&client->dev,
		"fw image size (%d) must be a multiple of 4 bytes\n",
		len);
		return -EINVAL;
	}
	else if (len > ISP_MAX_FW_SIZE) {
		dev_err(&client->dev,
		"fw image is too big, %d > %d\n", len, ISP_MAX_FW_SIZE);
		return -EINVAL;
	}

	dev_info(&client->dev, "fw download start\n");

	gpio_direction_output(info->pdata.gpio_vdd_en, 0);
	gpio_direction_output(info->pdata.gpio_sda, 0);
	gpio_direction_output(info->pdata.gpio_scl, 0);
	gpio_direction_output(info->pdata.gpio_resetb, 0);

	hw_reboot_bootloader(info);

	val = flash_readl(info, ISP_IC_INFO_ADDR);
	dev_info(&client->dev, "IC info: 0x%02x (%x)\n", val & 0xff, val);

	dev_info(&client->dev, "fw erase...\n");
	flash_erase(info);
	if (!flash_is_erased(info)) {
		ret = -ENXIO;
		goto err;
	}

	dev_info(&client->dev, "fw write...\n");
	/* XXX: what does this do?! */
	flash_writel(info, ISP_IC_INFO_ADDR, 0xffffff00 | (val & 0xff));
	usleep_range(1000, 1500);
	ret = fw_write_image(info, data, len);
	if (ret)
	goto err;
	usleep_range(1000, 1500);

	hw_reboot_normal(info);
	usleep_range(1000, 1500);
	dev_info(&client->dev, "fw download done...\n");
	return 0;

	err:
	dev_err(&client->dev, "fw download failed...\n");
	hw_reboot_normal(info);
	return ret;
}

static int get_fw_version(struct mms_ts_info *info) {
	int ret;
	int retries = 3;

	/* this seems to fail sometimes after a reset.. retry a few times */
	do { ret = i2c_smbus_read_byte_data(info->client, MMS_FW_VERSION); }
	while (ret < 0 && retries-- > 0);

	return ret;
}

static int mms_ts_enable(struct mms_ts_info *info) {
	mutex_lock(&info->lock);
	if (info->enabled)
	goto out;
	/* wake up the touch controller. */
	i2c_smbus_write_byte_data(info->client, 0, 0);
	usleep_range(3000, 5000);
	info->enabled = true;
	enable_irq(info->irq);
	out:
	mutex_unlock(&info->lock);
	return 0;
}

static void mms_ts_disable(struct mms_ts_info *info) {
	mutex_lock(&info->lock);
	if (!info->enabled)
	goto out;
	disable_irq(info->irq);
	i2c_smbus_write_byte_data(info->client, MMS_MODE_CONTROL, 0);
	usleep_range(10000, 12000);
	info->enabled = false;
	out:
	mutex_unlock(&info->lock);
}

static int mms_ts_input_open(struct input_dev *dev) {
	struct mms_ts_info *info = input_get_drvdata(dev);

	return mms_ts_enable(info);
}

static void mms_ts_input_close(struct input_dev *dev) {
	struct mms_ts_info *info = input_get_drvdata(dev);
	mms_ts_disable(info);
}

static int mms_ts_fw_load(const struct firmware *fw, void *context) {
	struct mms_ts_info *info = context;
	struct i2c_client *client = info->client;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int ret = 0;
	int ver;
	int retries = 3;
	struct mms_fw_image *fw_img;

	ver = get_fw_version(info);
	if (ver < 0) {
	ver = 0;
	dev_err(&client->dev,
	"can't read fw version, controller dead? forcing reflash\n");
}

	fw_img = (struct mms_fw_image *)fw->data;
	if (fw_img->hdr_len != sizeof(struct mms_fw_image) ||
	fw_img->data_len + fw_img->hdr_len != fw->size ||
	fw_img->hdr_ver != FW_HEADER_VERSION) {
		dev_err(&client->dev,
		"firmware image invalid\n");
		return -EINVAL;
	}

	if (ver == fw_img->fw_ver && !mms_force_reflash) {
		dev_info(&client->dev,
		"firmware is up-to-date, version 0x%02x\n", ver);
		return 0;
	}

	dev_info(&client->dev,
	"need fw update: current version 0x%02x, 0x%02x to flash%s\n",
	ver, fw_img->fw_ver, mms_force_reflash ? " forced" : "");

	while (retries--) {
		i2c_lock_adapter(adapter);
		info->pdata.mux_fw_flash(true);

		ret = fw_download(info, fw_img->data, fw_img->data_len);

		info->pdata.mux_fw_flash(false);
		i2c_unlock_adapter(adapter);

		if (ret < 0) {
			dev_err(&client->dev,
			"error updating firmware to version 0x%02x\n",
			fw_img->fw_ver);
			if (retries)
			dev_err(&client->dev, "retrying flashing\n");
			continue;
		}

		ver = get_fw_version(info);
		if (ver != fw_img->fw_ver) {
			dev_err(&client->dev,
			"fw update succeeded, but version mismatches "
			"(0x%x != 0x%x)\n",
			ver, fw_img->fw_ver);
			if (retries)
			dev_err(&client->dev, "retrying flashing\n");
			continue;
		}

		dev_info(&client->dev, "fw update done. ver = 0x%02x\n", ver);
		break;
	}

	if (ret < 0) {
		dev_err(&client->dev, "could not flash firmware, ran out of retries\n");
		return ret;
	}

	return 0;
}

static int __devinit mms_ts_config(struct mms_ts_info *info) {
	struct i2c_client *client = info->client;
	int ret;
	char path[32];
	const struct firmware *fw;

	mms_pwr_on_reset(info);

	snprintf(path, sizeof(path), "melfas/mms144_ts_rev%i.fw",
	info->pdata.fpcb_version);
	ret = request_firmware(&fw, path, &client->dev);
	if (ret) {
		dev_err(&client->dev, "error requesting firmware %s\n", path);
		return ret;
	}

	ret = mms_ts_fw_load(fw, info);
	if (ret < 0)
	dev_err(&client->dev, "failed to flash firmware %s\n", path);

	release_firmware(fw);
	return ret;
}

static int __devinit mms_ts_probe(struct i2c_client *client, const struct i2c_device_id *id) {
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct mms144_platform_data *pdata = client->dev.platform_data;
	struct mms_ts_info *info;
	struct input_dev *input_dev;
	int ret = 0;

	if (!pdata || !pdata->max_x || !pdata->max_y || !pdata->mux_fw_flash) {
		dev_err(&client->dev, "invalid platform data\n");
		return -EINVAL;
	}
	if (!client->irq) {
		dev_err(&client->dev, "invalid IRQ number\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
	return -EIO;

	info = kzalloc(sizeof(struct mms_ts_info), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!info || !input_dev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		goto err_alloc;
	}

	info->client = client;
	info->irq = client->irq;
	info->input_dev = input_dev;
	memcpy(&info->pdata, pdata, sizeof(*pdata));
	mutex_init(&info->lock);

	snprintf(info->phys, sizeof(info->phys),
	"%s/input0", dev_name(&client->dev));
	input_dev->name = "Melfas MMS144 Touchscreen";
	input_dev->phys = info->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	input_dev->open = mms_ts_input_open;
	input_dev->close = mms_ts_input_close;

	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
	0, info->pdata.max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
	0, info->pdata.max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, MAX_PRESSURE, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, MAX_WIDTH, 0, 0);
	ret = input_mt_init_slots(input_dev, MAX_FINGERS, INPUT_MT_DIRECT);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to allocate input slots\n");
		goto err_alloc;
	}

	input_set_drvdata(input_dev, info);

	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&client->dev, "failed to register input dev (%d)\n", ret);
		goto err_reg_input_dev;
	}

	i2c_set_clientdata(client, info);

	ret = mms_ts_config(info);
	if (ret)
	goto err_config;

	ret = request_threaded_irq(client->irq, NULL, mms_ts_interrupt,
	IRQF_TRIGGER_LOW | IRQF_ONESHOT,
	"mms144", info);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_req_irq;
	}
	disable_irq(client->irq);

	return 0;

	err_req_irq:
	err_config:
	input_unregister_device(input_dev);
	input_dev = NULL;
	err_reg_input_dev:
	if (input_dev)
	input_mt_destroy_slots(input_dev);
	err_alloc:
	input_free_device(input_dev);
	kfree(info);
	return ret;
}

static int __devexit mms_ts_remove(struct i2c_client *client) {
	struct mms_ts_info *info = i2c_get_clientdata(client);

	free_irq(info->irq, info);
	input_unregister_device(info->input_dev);
	kfree(info);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mms_ts_suspend(struct device *dev) {
	struct i2c_client *client = to_i2c_client(dev);
	struct mms_ts_info *info = i2c_get_clientdata(client);
	struct input_dev *input_dev = info->input_dev;
	int i;

	/* TODO: turn off the power (set vdd_en to 0) to the touchscreen on suspend */

	mutex_lock(&input_dev->mutex);
	if (input_dev->users)
	mms_ts_disable(info);
	mutex_unlock(&input_dev->mutex);

	for (i = 0; i < MAX_FINGERS; i++) {
		input_mt_slot(input_dev, i);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
	}
	input_mt_report_pointer_emulation(input_dev, false);
	input_sync(input_dev);

	return 0;
}

static int mms_ts_resume(struct device *dev) {
	struct i2c_client *client = to_i2c_client(dev);
	struct mms_ts_info *info = i2c_get_clientdata(client);
	struct input_dev *input_dev = info->input_dev;
	int ret = 0;

	mutex_lock(&input_dev->mutex);
	if (input_dev->users)
	ret = mms_ts_enable(info);
	mutex_unlock(&input_dev->mutex);

	return ret;
}
#endif

static SIMPLE_DEV_PM_OPS(mms_ts_pm_ops, mms_ts_suspend, mms_ts_resume);

static const struct i2c_device_id mms_ts_id[] = {
	{ "mms144", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mms_ts_id);

static struct i2c_driver mms_ts_driver = {
	.probe = mms_ts_probe,
	.remove = __devexit_p(mms_ts_remove),
	.driver = {
		.name = "mms144",
		.pm = &mms_ts_pm_ops,
	},
	.id_table = mms_ts_id,
};

module_i2c_driver(mms_ts_driver);
