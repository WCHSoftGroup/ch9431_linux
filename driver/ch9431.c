/*
 * CAN bus driver for CH9431 CAN Controller CH9431 with SPI Interface
 *
 * Copyright (C) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 * Web:      http://wch.cn
 * Author:   WCH <tech@wch.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * System required:
 * Kernel version beyond 4.0.x
 *
 * V1.0 - initial version
 * V1.1 - add support for kernel 3.11.x
 * V1.2 - re-implement spi TX/RX data transfer logic for bulk transfers
 * V1.3	- cancel interrupt delay to prevent duplicate reception
 */

#define DEBUG
#define VERBOSE_DEBUG

#undef DEBUG
#undef VERBOSE_DEBUG

#include <linux/can/core.h>
#include <linux/can/dev.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/ethtool.h>
#include <linux/freezer.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#define DRVNAME_CH9431 "ch9431"
#define DRIVER_AUTHOR "WCH"
#define DRIVER_DESC \
	"CAN bus driver for CH9431 CAN Controller with SPI Interface"
#define VERSION_DESC "V1.3 On 2025.06"

#define USE_IRQ_FROM_DTS
#define GPIO_NUMBER 0
//#undef USE_IRQ_FROM_DTS

#define REG_LABEL(REG) { #REG, REG }

#define CH9431_CLK_FREQ 20000000
#define TX_ECHO_SKB_MAX 1

#define CH9431_TXBD_CMD_LEN 1
#define CAN_FRAME_HEADER_LEN 5
#define CAN_FRAME_TX_CMD_LEN 6
#define CAN_FRAME_MAX_DATA_LEN 8
#define SPI_TRANSFER_BUF_LEN \
	(CAN_FRAME_TX_CMD_LEN + CAN_FRAME_MAX_DATA_LEN)

/* SPI Delay */
#define WAIT_DATA_US (1)
#define CLR_INTR_US (5)
#define OST_DELAY_MS (5)
#define RST_DELAY_MS (16)

/* CH9431 SPI commands */
#define CMD_CAN_WRITE 0x02
#define CMD_CAN_READ 0x03
#define CMD_CAN_BIT_MODIFY 0x05
#define CMD_CAN_RD_STATUS 0xA0
#define CMD_CAN_RX_STATUS 0xB0
#define CMD_CAN_RESET 0xC0
#define CMD_CAN_RTS 0x80
#define CMD_CAN_LOAD_TX 0X40
#define CMD_CAN_RD_RX_BUFF 0x90
#define INSTRUCTION_READ_RXBS(n) (((n) == 0) ? 0x90 : 0x94)
#define INSTRUCTION_READ_RXBD(n) (((n) == 0) ? 0x92 : 0x96)
#define INSTRUCTION_LOAD_TXBS(n) (0x40 + 2 * (n))
#define INSTRUCTION_LOAD_TXBD(n) (0x41 + 2 * (n))
#define INSTRUCTION_RTS(n) (0x80 | ((n) & 0x07))

/* CH9431 configuration registers */
#define CH9431_CTRL 0x0F
#define CH9431_CTRL_REQOP_MASK 0xE0
#define CH9431_CTRL_REQOP_NORMAL 0x00
#define CH9431_CTRL_REQOP_CONFIG 0x20
#define CH9431_CTRL_REQOP_LISTEN 0x40
#define CH9431_CTRL_REQOP_LOOPBACK 0x60
#define CH9431_CTRL_REQOP_SLEEP_LIGHT 0x80
#define CH9431_CTRL_REQOP_SLEEP_DEEP 0xA0
#define CH9431_CTRL_ABAT (1 << 4)
#define CH9431_CTRL_OSM (1 << 3)
#define CH9431_CTRL_CLKEN (1 << 2)
#define CH9431_CTRL_CLKPRE_20 0x00
#define CH9431_CTRL_CLKPRE_10 0x01
#define CH9431_CTRL_CLKPRE_08 0x02
#define CH9431_CTRL_CLKPRE_04 0x03
#define CH9431_STAT 0x0E
#define CH9431_STAT_OPMOD_NORMAL 0x00
#define CH9431_STAT_OPMOD_CONFIG 0x20
#define CH9431_STAT_OPMOD_LISTEN 0x40
#define CH9431_STAT_OPMOD_LOOPBACK 0x60
#define CH9431_STAT_OPMOD_UPDATE 0xC0
#define CH9431_STAT_ICOD_NONE 0x00
#define CH9431_STAT_ICOD_ERROR 0x02
#define CH9431_STAT_ICOD_WAKEUP 0x04
#define CH9431_STAT_ICOD_TXB0 0x06
#define CH9431_STAT_ICOD_TXB1 0x08
#define CH9431_STAT_ICOD_TXB2 0x0A
#define CH9431_STAT_ICOD_RXB0 0x0C
#define CH9431_STAT_ICOD_RXB1 0x0E
#define RXIPCTRL 0x0C
#define TXRTSCTRL 0x0D

#define TEC 0x1C
#define REC 0x1D

#define BTIMER3 0x28
#define BTIMER2 0x29
#define BTIMER2_SJW_SHIFT 4
#define BTIMER1 0x2A

#define CH9431_INTE 0x2B
#define CH9431_INTE_MERRE 0x80
#define CH9431_INTE_WAKIE 0x40
#define CH9431_INTE_ERRIE 0x20
#define CH9431_INTE_TX2IE 0x10
#define CH9431_INTE_TX1IE 0x08
#define CH9431_INTE_TX0IE 0x04
#define CH9431_INTE_RX1IE 0x02
#define CH9431_INTE_RX0IE 0x01
#define CH9431_INTF 0x2C
#define CH9431_INTF_MERRF 0x80
#define CH9431_INTF_WAKIF 0x40
#define CH9431_INTF_ERRIF 0x20
#define CH9431_INTF_TX2IF 0x10
#define CH9431_INTF_TX1IF 0x08
#define CH9431_INTF_TX0IF 0x04
#define CH9431_INTF_RX1IF 0x02
#define CH9431_INTF_RX0IF 0x01
#define CH9431_INTF_RX (CH9431_INTF_RX0IF | CH9431_INTF_RX1IF)
#define CH9431_INTF_TX \
	(CH9431_INTF_TX0IF | CH9431_INTF_TX1IF | CH9431_INTF_TX2IF)
#define CH9431_INTF_ERR (CH9431_INTF_ERRIF)
#define CH9431_EFLAG 0x2D
#define CH9431_EFLAG_RX1OVR (1 << 7)
#define CH9431_EFLAG_RX0OVR (1 << 6)
#define CH9431_EFLAG_TXBO (1 << 5)
#define CH9431_EFLAG_TXEP (1 << 4)
#define CH9431_EFLAG_RXEP (1 << 3)
#define CH9431_EFLAG_TXWAR (1 << 2)
#define CH9431_EFLAG_RXWAR (1 << 1)
#define CH9431_EFLAG_EWARN (1 << 0)

/*  CH9431 receive filters */
#define RXFAID(n) (((n) * 4) + 0x00)

#define RXF0SIDL 0x00
#define RXF0SIDH 0x01
#define RXF0EIDL 0x02
#define RXF0EIDH 0x03
#define RXFASIDL 0x23
#define RXFASIDH 0x48
#define RXFBSIDH 0x58
#define RXFAEIDL 0xD1
#define RXFAEIDH 0x48

#define RXF1SIDL 0x04
#define RXF1SIDH 0x05
#define RXF1EIDL 0x06
#define RXF1EIDH 0x07

#define RXF2SIDL 0x08
#define RXF2SIDH 0x09
#define RXF2EIDL 0x0A
#define RXF2EIDH 0x0B

#define RXFBID(n) (((n - 3) * 4) + 0x10)
#define RXF3SIDL 0x10
#define RXF3SIDH 0x11
#define RXF3EIDL 0x12
#define RXF3EIDH 0x13

#define RXF4SIDL 0x14
#define RXF4SIDH 0x15
#define RXF4EIDL 0x16
#define RXF4EIDH 0x17

#define RXF5SIDL 0x18
#define RXF5SIDH 0x19
#define RXF5EIDL 0x1A
#define RXF5EIDH 0x1B

/* Receive masks */
#define RXMID(n) (((n) * 4) + 0x20)
#define RXM0SIDL 0x20
#define RXM0SIDH 0x21
#define RXM0EIDL 0x22
#define RXM0EIDH 0x23

#define RXM1SIDL 0x24
#define RXM1SIDH 0x25
#define RXM1EIDL 0x26
#define RXM1EIDH 0x27

/* CH9431 TX buffer 0 */
#define TXBCTRL(n) (((n) * 0x10) + 0x30 + TXBCTRL_OFF)
#define TXB0CTRL 0x30
#define TXBS_LOAD_CMD 0
#define TXBCTRL_OFF 0
#define TXBSIDL_OFF 1
#define TXBSIDH_OFF 2
#define RXBSIDH_EXIDE 0x10
#define TXBEIDL_OFF 3
#define TXBEIDH_OFF 4
#define TXBDLC_OFF 5
#define RXBDLC_RTR 0x40
#define RXBDLC_LEN_MASK 0x0F
#define TXBD_LOAD_CMD 6
#define TXBDAT_OFF 7

#define TXB0SIDL 0x31
#define TXB0SIDH 0x32
#define TXB0EIDL 0x33
#define TXB0EIDH 0x34

#define TXB0DLC 0x35
#define TXBDAT(n) (((n) * 0x10) + 0x30 + TXBD_LOAD_CMD)
#define TXB0D0 0x36
#define TXB0D1 0x37
#define TXB0D2 0x38
#define TXB0D3 0x39
#define TXB0D4 0x3A
#define TXB0D5 0x3B
#define TXB0D6 0x3C
#define TXB0D7 0x3D

/* CH9431 TX buffer 1 */
#define TXB1CTRL 0x40

#define TXB1SIDL 0x41
#define TXB1SIDH 0x42
#define TXB1EIDL 0x43
#define TXB1EIDH 0x44

#define TXB1DLC 0x45
#define TXB1D0 0x46
#define TXB1D1 0x47
#define TXB1D2 0x48
#define TXB1D3 0x49
#define TXB1D4 0x4A
#define TXB1D5 0x4B
#define TXB1D6 0x4C
#define TXB1D7 0x4D

/* CH9431 TX buffer 2 */
#define TXB2CTRL 0x50

#define TXB2SIDL 0x51
#define TXB2SIDH 0x52
#define TXB2EIDL 0x53
#define TXB2EIDH 0x54

#define TXB2DLC 0x55
#define TXB2D0 0x56
#define TXB2D1 0x57
#define TXB2D2 0x58
#define TXB2D3 0x59
#define TXB2D4 0x5A
#define TXB2D5 0x5B
#define TXB2D6 0x5C
#define TXB2D7 0x5D

/* CH9431 RX buffer 0 */
#define RXBCTRL(n) (((n) * 0x10) + 0x60 + RXBCTRL_OFF)
#define RXB0CTRL 0x60
#define RXBCTRL_BUKT (1 << 2)
#define RXBSIDL_OFF 0
#define RXBSIDH_OFF 1
#define RXBSIDH_EXIDE 0x10
#define RXBEIDL_OFF 2
#define RXBEIDH_OFF 3
#define RXBDLC_OFF 4
#define RXBDLC_RTR 0x40
#define RXBDLC_LEN_MASK 0x0F
#define RXBDAT_OFF 5
#define RXBDAT(n) (((n) * 0x10) + 0x60 + RXBDAT_OFF)
#define RXB0SIDL 0x61
#define RXB0SIDH 0x62
#define RXB0EIDL 0x63
#define RXB0EIDH 0x64
#define RXB0DLC 0x65
#define RXB0D0 0x66

/* CH9431 RX buffer 1 */
#define RXB1CTRL 0x70

#define RXB1SIDL 0x71
#define RXB1SIDH 0x72
#define RXB1EIDL 0x73
#define RXB1EIDH 0x74

#define RXB1DLC 0x75
#define RXB1D0 0x76
#define RXB1D1 0x77
#define RXB1D2 0x78
#define RXB1D3 0x79
#define RXB1D4 0x7A
#define RXB1D5 0x7B
#define RXB1D6 0x7C
#define RXB1D7 0x7D

static const struct can_bittiming_const ch9431_bittiming_const = {
	.name = DRVNAME_CH9431,
	.tseg1_min = 2,
	.tseg1_max = 16,
	.tseg2_min = 1,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 256,
	.brp_inc = 1,
};

struct reg_label {
	char *name;
	unsigned int reg;
};

static struct reg_label reg_labels[] = {
	REG_LABEL(CH9431_CTRL), REG_LABEL(CH9431_STAT),
	REG_LABEL(BTIMER1),	REG_LABEL(BTIMER2),
	REG_LABEL(BTIMER3),	REG_LABEL(CH9431_INTE),
	REG_LABEL(CH9431_INTF), REG_LABEL(CH9431_EFLAG),
	REG_LABEL(TXB0CTRL),	REG_LABEL(TXB0DLC),
	REG_LABEL(TXB0D0),	REG_LABEL(TXB1CTRL),
	REG_LABEL(TXB1DLC),	REG_LABEL(TXB1D0),
	REG_LABEL(TXB2CTRL),	REG_LABEL(TXB2DLC),
	REG_LABEL(TXB2D0),	REG_LABEL(RXB0CTRL),
	REG_LABEL(RXB1CTRL),	REG_LABEL(RXF0SIDL),
	REG_LABEL(RXF0SIDH),	REG_LABEL(RXF0EIDL),
	REG_LABEL(RXF0EIDH),	REG_LABEL(RXF2SIDL),
	REG_LABEL(RXF2SIDH),	REG_LABEL(RXF2EIDL),
	REG_LABEL(RXF2EIDH),	REG_LABEL(RXF3SIDL),
	REG_LABEL(RXF3SIDH),	REG_LABEL(RXF3EIDL),
	REG_LABEL(RXF3EIDH),
};

struct ch9431_priv {
	struct can_priv can;
	struct net_device *ndev;
	struct spi_device *spi;
	struct mutex reg_lock;
	struct mutex ops_lock;

	u8 *spi_tx_buf;
	u8 *spi_rx_buf;

	struct sk_buff *tx_skb;
	struct workqueue_struct *wq;
	struct work_struct tx_work;
	struct work_struct restart_work;
	int force_quit;
	int after_suspend;
#define AFTER_SUSPEND_UP 1
#define AFTER_SUSPEND_DOWN 2
#define AFTER_SUSPEND_POWER 4
#define AFTER_SUSPEND_RESTART 8
	int restart_tx;
	bool tx_busy;
	struct regulator *power;
	struct regulator *transceiver;
};

static void ch9431_clean(struct net_device *ndev)
{
	struct ch9431_priv *ch9431 = netdev_priv(ndev);

	if (ch9431->tx_skb || ch9431->tx_busy)
		ndev->stats.tx_errors++;
	dev_kfree_skb(ch9431->tx_skb);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 13, 0))
	if (ch9431->tx_busy)
		can_free_echo_skb(ch9431->ndev, 0, NULL);
#else
	if (ch9431->tx_busy)
		can_free_echo_skb(ch9431->ndev, 0);
#endif
	ch9431->tx_skb = NULL;
	ch9431->tx_busy = false;
}

static int ch9431_spi_write(struct ch9431_priv *ch9431, u8 *buf, int len)
{
	int ret;

	mutex_lock(&ch9431->reg_lock);

	ret = spi_write(ch9431->spi, buf, len);
	if (ret)
		dev_err(&ch9431->spi->dev,
			"%s, spi write failed: ret = %d\n", __func__, ret);

	mutex_unlock(&ch9431->reg_lock);

	return ret;
}

static int ch9431_spi_trans(struct ch9431_priv *ch9431, u8 *buf, u8 reg,
			    int len)
{
	struct spi_transfer xfer[2] = {};
	struct spi_message m;
	u8 cmd = reg;
	int ret;

	xfer[0].tx_buf = &cmd;
	xfer[0].len = 1;
	xfer[0].cs_change = 0;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 19))
	xfer[0].delay.value = WAIT_DATA_US;
#else
	xfer[0].delay_usecs = WAIT_DATA_US;
#endif

	xfer[1].rx_buf = buf;
	xfer[1].len = len;

	mutex_lock(&ch9431->reg_lock);

	spi_message_init(&m);
	spi_message_add_tail(&xfer[0], &m);
	spi_message_add_tail(&xfer[1], &m);

	ret = spi_sync(ch9431->spi, &m);
	if (ret)
		dev_err(&ch9431->spi->dev,
			"%s, spi transfer failed: ret = %d\n", __func__,
			ret);

	mutex_unlock(&ch9431->reg_lock);

	return ret;
}

static u8 ch9431_read_reg(struct ch9431_priv *ch9431, u8 reg)
{
	struct spi_transfer xfer[2] = {};
	struct spi_message m;
	u8 cmd[2] = {};
	u8 val = 0;
	int ret;

	cmd[0] = CMD_CAN_READ;
	cmd[1] = reg;

	xfer[0].tx_buf = cmd;
	xfer[0].len = 2;
	xfer[0].cs_change = 0;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 19))
	xfer[0].delay.value = WAIT_DATA_US;
#else
	xfer[0].delay_usecs = WAIT_DATA_US;
#endif

	xfer[1].rx_buf = &val;
	xfer[1].len = 1;

	mutex_lock(&ch9431->reg_lock);

	spi_message_init(&m);
	spi_message_add_tail(&xfer[0], &m);
	spi_message_add_tail(&xfer[1], &m);

	ret = spi_sync(ch9431->spi, &m);
	if (ret)
		dev_err(&ch9431->spi->dev,
			"%s, spi transfer failed: ret = %d\n", __func__,
			ret);

	mutex_unlock(&ch9431->reg_lock);

	return val;
}

static void ch9431_read_2regs(struct ch9431_priv *ch9431, u8 reg, u8 *v1,
			      u8 *v2)
{
	*v1 = ch9431_read_reg(ch9431, reg);
	*v2 = ch9431_read_reg(ch9431, reg + 1);
}

static void ch9431_read_mem(struct ch9431_priv *ch9431, int rx_buf_idx,
			    u8 *buff, int len)
{
	ch9431_spi_trans(ch9431, buff, INSTRUCTION_READ_RXBD(rx_buf_idx),
			 len);
}

static int ch9431_write_reg(struct ch9431_priv *ch9431, u8 reg, u8 val)
{
	ch9431->spi_tx_buf[0] = CMD_CAN_WRITE;
	ch9431->spi_tx_buf[1] = reg;
	ch9431->spi_tx_buf[2] = val;

	return ch9431_spi_write(ch9431, ch9431->spi_tx_buf, 3);
}

static int ch9431_write_2regs(struct ch9431_priv *ch9431, u8 reg, u8 v1,
			      u8 v2)
{
	int ret;

	ret = ch9431_write_reg(ch9431, reg, v1);
	if (ret)
		dev_err(&ch9431->spi->dev,
			"%s, ch9431 write reg: [0x%02x] failed.\n",
			__func__, reg);

	ret = ch9431_write_reg(ch9431, reg + 1, v2);
	if (ret)
		dev_err(&ch9431->spi->dev,
			"%s, ch9431 write reg: [0x%02x] failed.\n",
			__func__, reg + 1);

	return ret;
}

static void ch9431_write_mem(struct ch9431_priv *ch9431, int tx_buf_idx,
			     u8 *buff, int len)
{
	ch9431_spi_write(ch9431, buff, len);
}

static int ch9431_write_bits(struct ch9431_priv *ch9431, u8 reg, u8 mask,
			      u8 val)
{
	struct spi_transfer xfer = {};
	struct spi_message m;
	int ret;

	ch9431->spi_tx_buf[0] = CMD_CAN_BIT_MODIFY;
	ch9431->spi_tx_buf[1] = reg;
	ch9431->spi_tx_buf[2] = mask;
	ch9431->spi_tx_buf[3] = val;

	xfer.tx_buf = ch9431->spi_tx_buf;
	xfer.len = 4;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 19))
	xfer.delay.value = CLR_INTR_US;
#else
	xfer.delay_usecs = CLR_INTR_US;
#endif

	mutex_lock(&ch9431->reg_lock);

	spi_message_init(&m);
	spi_message_add_tail(&xfer, &m);

	ret = spi_sync(ch9431->spi, &m);
	if (ret)
		dev_err(&ch9431->spi->dev,
			"%s, spi write bits failed: ret = %d\n", __func__,
			ret);

	mutex_unlock(&ch9431->reg_lock);

	return ret;
}

static u8 ch9431_read_stat(struct ch9431_priv *ch9431)
{
	u8 val;

	val = ch9431_read_reg(ch9431, CH9431_STAT);

	return val & CH9431_CTRL_REQOP_MASK;
}

static int ch9431_read_stat_poll_timeout(struct ch9431_priv *ch9431,
					 u8 mask, unsigned int delay_us,
					 unsigned int timeout_us)
{
	unsigned int elapsed_us = 0;
	u8 value;

	while (elapsed_us < timeout_us) {
		value = ch9431_read_stat(ch9431);

		if ((value & mask) || (mask == value))
			return 0;

		usleep_range(delay_us / 2, delay_us);
		elapsed_us += delay_us;
	}

	return -ETIMEDOUT;
}

static int ch9431_get_version(struct ch9431_priv *ch9431)
{
	u8 *val;
	int ret;

	val = kzalloc(8, GFP_KERNEL);

	ch9431->spi_tx_buf[0] = 0x00;
	ch9431->spi_tx_buf[1] = 0x76;

	ret = spi_write_then_read(ch9431->spi, ch9431->spi_tx_buf, 2, val,
				  8);

	printk(KERN_INFO
	       "ch9431 device probe, driver version: %s, fw version: %s\n",
	       VERSION_DESC, val);

	kfree(val);

	return ret;
}

static void ch9431_clear_intr(struct ch9431_priv *ch9431, u8 mask)
{
	ch9431_write_bits(ch9431, CH9431_INTF, mask, 0x00);
}

static const struct ethtool_ops ch9431_ethtool_ops = {
	.get_ts_info = ethtool_op_get_ts_info,
};

static void ch9431_hw_tx_frame(struct ch9431_priv *ch9431, u8 *buf,
			       int tx_buf_idx)
{
	ch9431_spi_write(ch9431, buf, CAN_FRAME_TX_CMD_LEN);
}

static void ch9431_hw_tx(struct ch9431_priv *ch9431,
			 struct can_frame *frame, int tx_buf_idx)
{
	u32 sid, eid, exide, rtr;
	u8 buf[SPI_TRANSFER_BUF_LEN + CH9431_TXBD_CMD_LEN];

	exide = (frame->can_id & CAN_EFF_FLAG) ?
			1 :
			0; /* Extended ID Enable */
	if (exide)
		sid = (frame->can_id & CAN_EFF_MASK) >> 18;
	else
		sid = frame->can_id & CAN_SFF_MASK; /* Standard ID */
	eid = frame->can_id & CAN_EFF_MASK; /* Extended ID */
	rtr = (frame->can_id & CAN_RTR_FLAG) ? 1 :
					       0; /* Remote transmission */

	buf[TXBS_LOAD_CMD] = INSTRUCTION_LOAD_TXBS(tx_buf_idx);
	buf[TXBSIDL_OFF] = (sid & 0xFF);
	buf[TXBSIDH_OFF] = ((eid & 0x03) << 6) | (exide << 4) |
			   ((sid >> 8) & 0x07);
	buf[TXBEIDL_OFF] = (eid >> 2) & 0xFF;
	buf[TXBEIDH_OFF] = (eid >> 10) & 0xFF;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0))
	buf[TXBDLC_OFF] = (rtr << 6) | (frame->len & 0x0F);
	buf[TXBD_LOAD_CMD] = INSTRUCTION_LOAD_TXBD(tx_buf_idx);

	memcpy(buf + TXBDAT_OFF, frame->data, frame->len);
	ch9431_hw_tx_frame(ch9431, buf, tx_buf_idx);
	ch9431_write_mem(ch9431, tx_buf_idx, buf + TXBD_LOAD_CMD,
			 frame->len + CH9431_TXBD_CMD_LEN);
#else
	buf[TXBDLC_OFF] = (rtr << 6) | (frame->can_dlc & 0x0F);
	buf[TXBD_LOAD_CMD] = INSTRUCTION_LOAD_TXBD(tx_buf_idx);

	memcpy(buf + TXBDAT_OFF, frame->data, frame->can_dlc);
	ch9431_hw_tx_frame(ch9431, buf, tx_buf_idx);
	ch9431_write_mem(ch9431, tx_buf_idx, buf + TXBD_LOAD_CMD,
			 frame->can_dlc + CH9431_TXBD_CMD_LEN);
#endif
	/* use INSTRUCTION_RTS, to avoid "repeated frame problem" */
	ch9431->spi_tx_buf[0] = INSTRUCTION_RTS(1 << tx_buf_idx);
	ch9431_spi_write(ch9431, ch9431->spi_tx_buf, 1);
}

static void ch9431_hw_rx_frame(struct ch9431_priv *ch9431, u8 *buf,
			       int rx_buf_idx)
{
	ch9431_spi_trans(ch9431, buf, INSTRUCTION_READ_RXBS(rx_buf_idx),
			 CAN_FRAME_HEADER_LEN);
}

static void ch9431_hw_rx(struct ch9431_priv *ch9431, int rx_buf_idx)
{
	struct net_device *ndev = ch9431->ndev;
	struct sk_buff *skb;
	struct can_frame *frame;
	u8 buf[CAN_FRAME_HEADER_LEN];

	skb = alloc_can_skb(ndev, &frame);
	if (!skb) {
		dev_err(&ch9431->spi->dev, "%s, cannot allocate RX skb\n",
			__func__);
		ch9431->ndev->stats.rx_dropped++;
		return;
	}

	ch9431_hw_rx_frame(ch9431, buf, rx_buf_idx);

	if (buf[RXBSIDH_OFF] & RXBSIDH_EXIDE) {
		/* Extended ID format */
		frame->can_id = CAN_EFF_FLAG;
		frame->can_id |=
			/* Extended ID part */
			(buf[RXBEIDH_OFF] << 10) |
			(buf[RXBEIDL_OFF] << 2) |
			((buf[RXBSIDH_OFF] >> 6) & 0x03) |
			/* Standard ID part */
			((((buf[RXBSIDH_OFF] & 0x07) << 8) |
			  buf[RXBSIDL_OFF])
			 << 18);
		/* Remote transmission request */
		if (buf[RXBDLC_OFF] & RXBDLC_RTR)
			frame->can_id |= CAN_RTR_FLAG;
	} else {
		/* Standard ID format */
		frame->can_id = ((((buf[RXBSIDH_OFF] & 0x07) << 8) |
				  buf[RXBSIDL_OFF]));
	}

	/* Data length */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0))
	frame->len = can_cc_dlc2len(buf[RXBDLC_OFF] & RXBDLC_LEN_MASK);
	if (!(frame->can_id & CAN_RTR_FLAG)) {
		ch9431_read_mem(ch9431, rx_buf_idx, frame->data,
				frame->len);

		ch9431->ndev->stats.rx_bytes += frame->len;
	}
#else
	frame->can_dlc = get_can_dlc(buf[RXBDLC_OFF] & RXBDLC_LEN_MASK);
	if (!(frame->can_id & CAN_RTR_FLAG)) {
		ch9431_read_mem(ch9431, rx_buf_idx, frame->data,
				frame->can_dlc);

		ch9431->ndev->stats.rx_bytes += frame->can_dlc;
	}
#endif

	ch9431->ndev->stats.rx_packets++;

	netif_rx(skb);
}

static int ch9431_power_enable(struct regulator *reg, int enable)
{
	if (IS_ERR_OR_NULL(reg))
		return 0;

	if (enable)
		return regulator_enable(reg);
	else
		return regulator_disable(reg);
}

static void ch9431_hw_sleep(struct ch9431_priv *ch9431)
{
	ch9431_write_reg(ch9431, CH9431_CTRL,
			 CH9431_CTRL_REQOP_SLEEP_LIGHT);
}

/* May only be called when device is sleeping! */
static int ch9431_hw_wake(struct ch9431_priv *ch9431)
{
	struct spi_device *spi = ch9431->spi;
	u8 value;
	int ret;

	/* Force wakeup interrupt to wake device, but don't execute IST */
	disable_irq(spi->irq);
	ch9431_write_2regs(ch9431, CH9431_INTE, CH9431_INTE_WAKIE,
			   CH9431_INTF_WAKIF);

	/* Wait for oscillator startup timer after wake up */
	mdelay(OST_DELAY_MS);

	/* Put device into config mode */
	ch9431_write_reg(ch9431, CH9431_CTRL, CH9431_CTRL_REQOP_CONFIG);

	/* Wait for the device to enter config mode */
	value = ch9431_read_reg(ch9431, CH9431_CTRL);

	if (value != 0x20) {
		dev_err(&spi->dev,
			"%s, ch9431 didn't enter in config mode\n",
			__func__);
		return ret;
	}

	/* Disable and clear pending interrupts */
	ch9431_write_2regs(ch9431, CH9431_INTE, 0x00, 0x00);
	enable_irq(spi->irq);

	return 0;
}

static int ch9431_set_normal_mode(struct ch9431_priv *ch9431)
{
	struct spi_device *spi = ch9431->spi;
	int ret;

	/* Enable interrupts */
	ch9431_write_reg(ch9431, CH9431_INTE,
			 CH9431_INTE_RX0IE | CH9431_INTE_RX1IE |
				 CH9431_INTE_ERRIE | CH9431_INTE_TX0IE |
				 CH9431_INTE_TX1IE | CH9431_INTE_TX2IE);

	if (ch9431->can.ctrlmode & CAN_CTRLMODE_LOOPBACK) {
		/* Put device into loopback mode */
		ch9431_write_reg(ch9431, CH9431_CTRL,
				 CH9431_CTRL_REQOP_LOOPBACK);
	} else if (ch9431->can.ctrlmode & CAN_CTRLMODE_LISTENONLY) {
		/* Put device into listen-only mode */
		ch9431_write_reg(ch9431, CH9431_CTRL,
				 CH9431_CTRL_REQOP_LISTEN);
	} else {
		/* Put device into normal mode */
		ch9431_write_reg(ch9431, CH9431_CTRL,
				 CH9431_CTRL_REQOP_NORMAL);

		/* Wait for the device to enter normal mode */
		ret = ch9431_read_stat_poll_timeout(
			ch9431, 0, OST_DELAY_MS * 1000, 10000);
		if (ret) {
			dev_err(&spi->dev,
				"%s, ret = %d, ch9431 didn't enter in normal mode!\n",
				__func__, ret);
			return ret;
		}
	}

	ch9431->can.state = CAN_STATE_ERROR_ACTIVE;
	return 0;
}

/* Set CNF1/CNF2/CNF3 Set Baud Rate*/
static int ch9431_do_set_bittiming(struct net_device *ndev)
{
	struct ch9431_priv *ch9431 = netdev_priv(ndev);
	struct can_bittiming *bt = &ch9431->can.bittiming;
	u8 cfg1, cfg2, cfg3;

	cfg1 = (bt->brp - 1);
	cfg2 = (((bt->sjw - 1) << 4) |
		(bt->prop_seg + bt->phase_seg1 - 1));
	cfg3 = (bt->phase_seg2 - 1);

	ch9431_write_reg(ch9431, BTIMER1, cfg1);
	ch9431_write_reg(ch9431, BTIMER2, cfg2);
	ch9431_write_reg(ch9431, BTIMER3, cfg3);

	dev_dbg(&ch9431->spi->dev, "%s, CNF: 0x%02x 0x%02x 0x%02x\n",
		__func__, ch9431_read_reg(ch9431, BTIMER1),
		ch9431_read_reg(ch9431, BTIMER2),
		ch9431_read_reg(ch9431, BTIMER3));

	return 0;
}

static int ch9431_set_rxfilter(struct ch9431_priv *ch9431, u8 reg, u8 sidl,
			       u8 sidh, u8 eidl, u8 eidh)
{
	int ret, i;
	u8 val[4] = { sidl, sidh, eidl, eidh };

	for (i = 0; i < 4; i++) {
		ret = ch9431_write_reg(ch9431, reg + i, val[i]);
		if (ret) {
			dev_err(&ch9431->spi->dev,
				"%s, ch9431 write reg: [0x%02x] failed.\n",
				__func__, reg + i);
			return ret;
		}
	}

	return ret;
}

static int ch9431_set_rxmask(struct ch9431_priv *ch9431, int rxm_id,
			     u8 val)
{
	int ret, i;

	for (i = 0; i < 4; i++) {
		ret = ch9431_write_reg(ch9431, RXMID(rxm_id) + i, val);
		if (ret) {
			dev_err(&ch9431->spi->dev,
				"%s, ch9431 write reg: [0x%02x] failed.\n",
				__func__, RXMID(rxm_id) + i);
			return ret;
		}
	}

	return ret;
}

static int ch9431_setup(struct ch9431_priv *ch9431)
{
	struct net_device *ndev = ch9431->ndev;
	int i;

	ch9431_write_reg(ch9431, RXB0CTRL, RXBCTRL_BUKT);

	ch9431_set_rxmask(ch9431, 0, 0x00);
	ch9431_set_rxmask(ch9431, 1, 0x00);

	for (i = 0; i < 5; i++) {
		if (i < 3)
			ch9431_set_rxfilter(ch9431, RXFAID(i), RXFASIDL,
					    RXFASIDH, RXFAEIDL, RXFAEIDH);
		else
			ch9431_set_rxfilter(ch9431, RXFBID(i), RXFASIDL,
					    RXFBSIDH, RXFAEIDL, RXFAEIDH);
	}

	ch9431_do_set_bittiming(ndev);

	return 0;
}

static void ch9431_tx_delay(struct work_struct *work)
{
	struct ch9431_priv *ch9431 =
		container_of(work, struct ch9431_priv, tx_work);
	struct net_device *ndev = ch9431->ndev;
	struct can_frame *frame;

	mutex_lock(&ch9431->ops_lock);

	if (ch9431->tx_skb) {
		if (ch9431->can.state == CAN_STATE_BUS_OFF) {
			ch9431_clean(ndev);
		} else {
			frame = (struct can_frame *)ch9431->tx_skb->data;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0))
			if (frame->len > CAN_FRAME_MAX_DATA_LEN)
				frame->len = CAN_FRAME_MAX_DATA_LEN;
#else
			if (frame->can_dlc > CAN_FRAME_MAX_DATA_LEN)
				frame->can_dlc = CAN_FRAME_MAX_DATA_LEN;
#endif
			ch9431_hw_tx(ch9431, frame, 0);
			ch9431->tx_busy = true;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0))
			can_put_echo_skb(ch9431->tx_skb, ndev, 0, 0);
#else
			can_put_echo_skb(ch9431->tx_skb, ndev, 0);
#endif
			ch9431->tx_skb = NULL;
		}
	}

	mutex_unlock(&ch9431->ops_lock);
}

static int ch9431_hw_reset(struct ch9431_priv *ch9431)
{
	struct spi_device *spi = ch9431->spi;
	int ret;

	/* Wait for oscillator startup timer after power up */
	mdelay(RST_DELAY_MS);

	ch9431->spi_tx_buf[0] = CMD_CAN_RESET;
	ret = ch9431_spi_write(ch9431, ch9431->spi_tx_buf, 1);
	if (ret) {
		dev_err(&spi->dev, "CH9431 CAN reset cmd send fail.\n");
		return ret;
	}

	/* Wait for oscillator startup timer after reset */
	mdelay(RST_DELAY_MS);

	/* Wait for reset to finish */
	ret = ch9431_read_stat_poll_timeout(ch9431,
					    CH9431_CTRL_REQOP_CONFIG,
					    OST_DELAY_MS * 1000, 10000);
	if (ret)
		dev_err(&spi->dev,
			"%s, ch9431 didn't enter in conf mode after reset\n",
			__func__);
	return ret;
}

static void ch9431_error_skb(struct net_device *ndev, int can_id,
			     int data1)
{
	struct sk_buff *skb;
	struct can_frame *frame;

	skb = alloc_can_err_skb(ndev, &frame);
	if (skb) {
		frame->can_id |= can_id;
		frame->data[1] = data1;
		netif_rx(skb);
	} else {
		netdev_err(ndev, "%s, cannot allocate error skb\n",
			   __func__);
	}
}

static void ch9431_restart_tx(struct work_struct *work)
{
	struct ch9431_priv *ch9431 =
		container_of(work, struct ch9431_priv, restart_work);
	struct net_device *ndev = ch9431->ndev;

	mutex_lock(&ch9431->ops_lock);
	if (ch9431->after_suspend) {
		if (ch9431->after_suspend & AFTER_SUSPEND_POWER) {
			ch9431_hw_reset(ch9431);
			ch9431_setup(ch9431);
		} else {
			ch9431_hw_wake(ch9431);
		}
		ch9431->force_quit = 0;
		if (ch9431->after_suspend & AFTER_SUSPEND_RESTART) {
			ch9431_set_normal_mode(ch9431);
		} else if (ch9431->after_suspend & AFTER_SUSPEND_UP) {
			netif_device_attach(ndev);
			ch9431_clean(ndev);
			ch9431_set_normal_mode(ch9431);
			netif_wake_queue(ndev);
		} else {
			ch9431_hw_sleep(ch9431);
		}
		ch9431->after_suspend = 0;
	}

	if (ch9431->restart_tx) {
		ch9431->restart_tx = 0;
		ch9431_write_reg(ch9431, TXBCTRL(0), 0);
		ch9431_clean(ndev);
		netif_wake_queue(ndev);
		ch9431_error_skb(ndev, CAN_ERR_RESTARTED, 0);
	}
	mutex_unlock(&ch9431->ops_lock);
}

static irqreturn_t ch9431_rx_threaded_irq(int irq, void *pw)
{
	struct ch9431_priv *ch9431 = pw;
	struct net_device *ndev = ch9431->ndev;

	mutex_lock(&ch9431->ops_lock);

	if (!ch9431->force_quit) {
		enum can_state new_state;
		u8 intf, errf;
		u8 clear_intf = 0;
		int can_id = 0, data1 = 0;

		ch9431_read_2regs(ch9431, CH9431_INTF, &intf, &errf);

		/* Receive buffer 0 */
		if (intf & CH9431_INTF_RX0IF) {
			ch9431_hw_rx(ch9431, 0);
			ch9431_clear_intr(ch9431, CH9431_INTF_RX0IF);

			/* Check if buffer 1 is already known to be full, no need to re-read */
			if (!(intf & CH9431_INTF_RX1IF)) {
				u8 intf1, errf1;

				/* Intf reg needs to be read again to avoid a race condition */
				ch9431_read_2regs(ch9431, CH9431_INTF,
						  &intf1, &errf1);

				/* Combine flags from both operations for error handling */
				intf |= intf1;
				errf |= errf1;
			}
		}

		/* Receive buffer 1 */
		if (intf & CH9431_INTF_RX1IF) {
			ch9431_hw_rx(ch9431, 1);
			ch9431_clear_intr(ch9431, CH9431_INTF_RX1IF);
		}

		/* Mask out flags we don't care about */
		intf &= CH9431_INTF_RX | CH9431_INTF_TX | CH9431_INTF_ERR;

		/* Any error or tx interrupt we need to clear? */
		if (intf & (CH9431_INTF_ERR | CH9431_INTF_TX))
			clear_intf |= intf &
				      (CH9431_INTF_ERR | CH9431_INTF_TX);
		if (clear_intf)
			ch9431_clear_intr(ch9431, clear_intf);

		if (errf & (CH9431_EFLAG_RX0OVR | CH9431_EFLAG_RX1OVR))
			ch9431_write_bits(ch9431, CH9431_EFLAG, errf,
					  0x00);

		/* Update can state */
		if (errf & CH9431_EFLAG_TXBO) {
			new_state = CAN_STATE_BUS_OFF;
			can_id |= CAN_ERR_BUSOFF;
		} else if (errf & CH9431_EFLAG_TXEP) {
			new_state = CAN_STATE_ERROR_PASSIVE;
			can_id |= CAN_ERR_CRTL;
			data1 |= CAN_ERR_CRTL_TX_PASSIVE;
		} else if (errf & CH9431_EFLAG_RXEP) {
			new_state = CAN_STATE_ERROR_PASSIVE;
			can_id |= CAN_ERR_CRTL;
			data1 |= CAN_ERR_CRTL_RX_PASSIVE;
		} else if (errf & CH9431_EFLAG_TXWAR) {
			new_state = CAN_STATE_ERROR_WARNING;
			can_id |= CAN_ERR_CRTL;
			data1 |= CAN_ERR_CRTL_TX_WARNING;
		} else if (errf & CH9431_EFLAG_RXWAR) {
			new_state = CAN_STATE_ERROR_WARNING;
			can_id |= CAN_ERR_CRTL;
			data1 |= CAN_ERR_CRTL_RX_WARNING;
		} else {
			new_state = CAN_STATE_ERROR_ACTIVE;
		}

		/* Update can state statistics */
		switch (ch9431->can.state) {
		case CAN_STATE_ERROR_ACTIVE:
			if (new_state >= CAN_STATE_ERROR_WARNING &&
			    new_state <= CAN_STATE_BUS_OFF)
				ch9431->can.can_stats.error_warning++;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
			fallthrough;
#endif
		case CAN_STATE_ERROR_WARNING: /* fallthrough */
			if (new_state >= CAN_STATE_ERROR_PASSIVE &&
			    new_state <= CAN_STATE_BUS_OFF)
				ch9431->can.can_stats.error_passive++;
			break;
		default:
			break;
		}
		ch9431->can.state = new_state;

		if (intf & CH9431_INTF_ERRIF) {
			/* Handle overflow counters */
			if (errf &
			    (CH9431_EFLAG_RX0OVR | CH9431_EFLAG_RX1OVR)) {
				if (errf & CH9431_EFLAG_RX0OVR) {
					ndev->stats.rx_over_errors++;
					ndev->stats.rx_errors++;
				}
				if (errf & CH9431_EFLAG_RX1OVR) {
					ndev->stats.rx_over_errors++;
					ndev->stats.rx_errors++;
				}
				can_id |= CAN_ERR_CRTL;
				data1 |= CAN_ERR_CRTL_RX_OVERFLOW;
			}
			ch9431_error_skb(ndev, can_id, data1);
		}

		if (ch9431->can.state == CAN_STATE_BUS_OFF) {
			if (ch9431->can.restart_ms == 0) {
				ch9431->force_quit = 1;
				ch9431->can.can_stats.bus_off++;
				can_bus_off(ndev);
				ch9431_hw_sleep(ch9431);
				goto out;
			}
		}

		if (intf == 0)
			goto out;

		if (intf & CH9431_INTF_TX) {
			if (ch9431->tx_busy) {
				ndev->stats.tx_packets++;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0))
				ndev->stats.tx_bytes +=
					can_get_echo_skb(ndev, 0, NULL);
#else
				ndev->stats.tx_bytes +=
					can_get_echo_skb(ndev, 0);
#endif
				ch9431->tx_busy = false;
			}
			netif_wake_queue(ndev);
		}
	}

out:
	mutex_unlock(&ch9431->ops_lock);

	return IRQ_HANDLED;
}

/*
 * Open can device
 * Called when the can device is marked active, such as a user executing
 * 'ifconfig candev up' on the device
 */
static int ch9431_open(struct net_device *ndev)
{
	struct ch9431_priv *ch9431 = netdev_priv(ndev);
	struct spi_device *spi = ch9431->spi;
	int ret;

	ret = open_candev(ndev);
	if (ret) {
		dev_err(&spi->dev, "%s, Unable to init baudrate!\n",
			__func__);
		return ret;
	}

	mutex_lock(&ch9431->ops_lock);

	ch9431_power_enable(ch9431->transceiver, 1);

	ch9431->force_quit = 0;
	ch9431->tx_skb = NULL;
	ch9431->tx_busy = false;

	ret = ch9431_hw_wake(ch9431);
	if (ret)
		goto out_close;

	ret = ch9431_setup(ch9431);
	if (ret)
		goto out_close;

	ret = ch9431_set_normal_mode(ch9431);
	if (ret)
		goto out_close;

	netif_wake_queue(ndev);
	mutex_unlock(&ch9431->ops_lock);

	return 0;

out_close:
	ch9431_hw_sleep(ch9431);
	ch9431_power_enable(ch9431->transceiver, 0);
	close_candev(ndev);
	netdev_err(ndev, "%s : ch9431 open failed, ret: %d\n", __func__,
		   ret);
	mutex_unlock(&ch9431->ops_lock);
	return ret;
}

static int ch9431_hw_probe(struct ch9431_priv *ch9431)
{
	struct spi_device *spi = ch9431->spi;
	u8 ctrl;
	int ret;

	ret = ch9431_hw_reset(ch9431);
	if (ret)
		return ret;

	ctrl = ch9431_read_reg(ch9431, CH9431_CTRL);

	dev_dbg(&spi->dev, "%s, CH9431_CTRL 0x%02x\n", __func__, ctrl);

	/* Check for power up default value */
	if ((ctrl & 0x17) != 0x07)
		return -ENODEV;

	ch9431_get_version(ch9431);

	return 0;
}

/*
 * Close can device
 * Called to close down a network device which has been active. Cancel any
 * work, shutdown the RX and TX process and then place the chip into a low
 * power state while it is not being used
 */
static int ch9431_stop(struct net_device *ndev)
{
	struct ch9431_priv *ch9431 = netdev_priv(ndev);

	close_candev(ndev);
	ch9431->force_quit = 1;

	mutex_lock(&ch9431->ops_lock);

	/* Disable and clear pending interrupts */
	ch9431_write_2regs(ch9431, CH9431_INTE, 0x00, 0x00);

	ch9431_write_reg(ch9431, TXBCTRL(0), 0);
	ch9431_clean(ndev);

	ch9431_hw_sleep(ch9431);
	ch9431_power_enable(ch9431->transceiver, 0);
	ch9431->can.state = CAN_STATE_STOPPED;

	mutex_unlock(&ch9431->ops_lock);

	return 0;
}

/*
 * event: play a schedule starter in condition
 */
static netdev_tx_t ch9431_start_xmit(struct sk_buff *skb,
				     struct net_device *ndev)
{
	struct ch9431_priv *ch9431 = netdev_priv(ndev);

	if (ch9431->tx_skb || ch9431->tx_busy) {
		dev_warn(&ch9431->spi->dev,
			 "hard_xmit called while tx busy\n");
		return NETDEV_TX_BUSY;
	}

	if (can_dropped_invalid_skb(ndev, skb))
		return NETDEV_TX_OK;

	netif_stop_queue(ndev);
	ch9431->tx_skb = skb;
	queue_work(ch9431->wq, &ch9431->tx_work);

	return NETDEV_TX_OK;
}

static int ch9431_do_set_mode(struct net_device *ndev, enum can_mode mode)
{
	struct ch9431_priv *ch9431 = netdev_priv(ndev);

	switch (mode) {
	case CAN_MODE_START:
		ch9431_clean(ndev);
		/* We have to delay work since SPI I/O may sleep */
		ch9431->can.state = CAN_STATE_ERROR_ACTIVE;
		ch9431->restart_tx = 1;
		if (ch9431->can.restart_ms == 0)
			ch9431->after_suspend = AFTER_SUSPEND_RESTART;
		queue_work(ch9431->wq, &ch9431->restart_work);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static const struct net_device_ops ch9431_netdev_ops = {
	.ndo_open = ch9431_open,
	.ndo_stop = ch9431_stop,
	.ndo_start_xmit = ch9431_start_xmit,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 15, 0))
	.ndo_change_mtu = can_change_mtu,
#endif
};

static ssize_t reg_dump_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct ch9431_priv *ch9431 = NULL;
	int i, len = 0;
	u8 val;

	dev_info(dev, "reg_dump_show");
	if (!spi) {
		dev_err(dev, "%s, spi_device is NULL\n", __func__);
		return -EINVAL;
	}

	ch9431 = dev_get_drvdata(dev);

	if (!ch9431) {
		dev_err(dev, "%s, ch9431 priv is NULL\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < sizeof(reg_labels) / sizeof(reg_labels[0]); i++) {
		val = ch9431_read_reg(ch9431, reg_labels[i].reg);
		len += sprintf(buf + len, "%s: 0x%02x\n",
			       reg_labels[i].name, val);
	}

	return len;
}

static ssize_t reg_dump_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct ch9431_priv *ch9431;
	u8 reg;
	u8 val;
	char reg_name[32];

	dev_info(dev, "reg_dump_store\n");
	if (!spi) {
		dev_err(dev, "%s, spi_device is NULL\n", __func__);
		return -EINVAL;
	}

	ch9431 = dev_get_drvdata(dev);

	if (sscanf(buf, "%31s %02hhx", reg_name, &val) == 2) {
		int i;

		for (i = 0; i < sizeof(reg_labels) / sizeof(reg_labels[0]);
		     i++) {
			if (strcmp(reg_labels[i].name, reg_name) == 0) {
				reg = reg_labels[i].reg;
				if (ch9431_write_reg(ch9431, reg, val) < 0)
					dev_info(
						dev,
						"set reg: 0x%02x - value: 0x%02x failed!\n",
						reg, val);
				else
					dev_info(
						dev,
						"set reg: 0x%02x - value: 0x%02x success!\n",
						reg, val);
				break;
			}
		}
	}

	return count;
}

static DEVICE_ATTR(reg_dump, S_IRUGO | S_IWUSR, reg_dump_show,
		   reg_dump_store);

static struct attribute *ch9431_attributes[] = { &dev_attr_reg_dump.attr,
						 NULL };

static struct attribute_group ch9431_attribute_group = {
	.attrs = ch9431_attributes
};

int ch9431_create_sysfs(struct spi_device *spi)
{
	int err;

	err = sysfs_create_group(&spi->dev.kobj, &ch9431_attribute_group);
	if (err != 0) {
		dev_err(&spi->dev, "sysfs_create_group() failed!");
		sysfs_remove_group(&spi->dev.kobj,
				   &ch9431_attribute_group);
		return -EIO;
	}

	err = sysfs_create_link(NULL, &spi->dev.kobj, "ch9431");
	if (err < 0) {
		dev_err(&spi->dev, "Failed to create link!");
		return -EIO;
	}

	dev_info(&spi->dev, "sysfs_create_group() successful!");

	return err;
}

static int ch9431_request_irq(struct ch9431_priv *ch9431)
{
	struct net_device *ndev = ch9431->ndev;
	struct spi_device *spi = ch9431->spi;
	int ret;
	unsigned long flags = IRQF_TRIGGER_LOW;

	/* if your platform supports acquire irq number from dts */
#ifdef USE_IRQ_FROM_DTS
	ndev->irq = spi->irq; /* by dts */
#else
	ret = devm_gpio_request(&spi->dev, GPIO_NUMBER, "gpioint");
	if (ret) {
		dev_err(&spi->dev, "gpio_request fail.\n");
		goto out;
	}

	ret = gpio_direction_input(GPIO_NUMBER);
	if (ret) {
		dev_err(&spi->dev, "gpio_direction_input fail.\n");
		goto out;
	}
	irq_set_irq_type(gpio_to_irq(GPIO_NUMBER), flags);

	spi->irq = gpio_to_irq(GPIO_NUMBER);
	ndev->irq = spi->irq;
#endif
	ret = request_threaded_irq(spi->irq, NULL, ch9431_rx_threaded_irq,
				   flags | IRQF_ONESHOT,
				   dev_name(&spi->dev), ch9431);
	if (ret) {
		dev_err(&spi->dev, "failed to acquire irq %d\n", spi->irq);
		goto out;
	}

out:
	return ret;
}

static int ch9431_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct net_device *ndev;
	struct ch9431_priv *ch9431;
	int ret = 0;

	/* Allocate can/net device */
	ndev = alloc_candev(sizeof(struct ch9431_priv), TX_ECHO_SKB_MAX);
	if (!ndev)
		return -ENOMEM;

	ndev->netdev_ops = &ch9431_netdev_ops;
	ndev->ethtool_ops = &ch9431_ethtool_ops;
	ndev->flags |= IFF_ECHO;

	ch9431 = netdev_priv(ndev);
	ch9431->can.bittiming_const = &ch9431_bittiming_const;
	ch9431->can.do_set_mode = ch9431_do_set_mode;
	ch9431->can.clock.freq = CH9431_CLK_FREQ;
	ch9431->can.ctrlmode_supported = CAN_CTRLMODE_3_SAMPLES |
					 CAN_CTRLMODE_LOOPBACK |
					 CAN_CTRLMODE_LISTENONLY;
	ch9431->ndev = ndev;
	ch9431->spi = spi;

	spi_set_drvdata(spi, ch9431);

	/* Configure the SPI bus */
	spi->bits_per_word = 8;
	spi->max_speed_hz = spi->max_speed_hz ?: 12 * 1000 * 1000;
	ret = spi_setup(spi);
	if (ret)
		goto out_free;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 12, 0))
	ch9431->power = devm_regulator_get_optional(dev, "vdd");
	ch9431->transceiver = devm_regulator_get_optional(dev, "xceiver");
#else
	ch9431->power = devm_regulator_get(dev, "vdd");
	ch9431->transceiver = devm_regulator_get(dev, "xceiver");
#endif

	if ((PTR_ERR(ch9431->power) == -EPROBE_DEFER) ||
	    (PTR_ERR(ch9431->transceiver) == -EPROBE_DEFER)) {
		ret = -EPROBE_DEFER;
		goto out_free;
	}

	ret = ch9431_power_enable(ch9431->power, 1);
	if (ret)
		goto out_free;

	ch9431->wq = alloc_workqueue("ch9431_wq",
				     WQ_FREEZABLE | WQ_MEM_RECLAIM, 0);
	if (!ch9431->wq) {
		ret = -ENOMEM;
		goto out_free;
	}

	INIT_WORK(&ch9431->tx_work, ch9431_tx_delay);
	INIT_WORK(&ch9431->restart_work, ch9431_restart_tx);

	mutex_init(&ch9431->ops_lock);
	mutex_init(&ch9431->reg_lock);

	ch9431->spi_tx_buf =
		devm_kzalloc(&spi->dev, SPI_TRANSFER_BUF_LEN, GFP_KERNEL);
	if (!ch9431->spi_tx_buf) {
		ret = -ENOMEM;
		goto error_probe;
	}

	ch9431->spi_rx_buf =
		devm_kzalloc(&spi->dev, SPI_TRANSFER_BUF_LEN, GFP_KERNEL);
	if (!ch9431->spi_rx_buf) {
		ret = -ENOMEM;
		goto error_probe;
	}

	SET_NETDEV_DEV(ndev, dev);

	ret = ch9431_hw_probe(ch9431);
	if (ret) {
		if (ret == -ENODEV)
			dev_err(dev,
				"Cannot initialize ch9431. Wrong wiring?\n");
		goto error_probe;
	}

	ch9431_hw_sleep(ch9431);

	ret = register_candev(ndev);
	if (ret)
		goto out_unregister_candev;

	ch9431_create_sysfs(spi);

	ch9431_request_irq(ch9431);

	return 0;

out_unregister_candev:
	unregister_candev(ndev);

error_probe:
	destroy_workqueue(ch9431->wq);
	ch9431->wq = NULL;
	ch9431_power_enable(ch9431->power, 0);

out_free:
	free_candev(ndev);

	dev_err(dev, "ch9431_probe failed, err=%d\n", -ret);
	return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
static void ch9431_drv_remove(struct spi_device *spi)
{
	struct ch9431_priv *ch9431 = spi_get_drvdata(spi);
	struct net_device *net = ch9431->ndev;

	unregister_candev(net);

	ch9431_power_enable(ch9431->power, 0);

	destroy_workqueue(ch9431->wq);
	ch9431->wq = NULL;

	free_irq(ch9431->spi->irq, ch9431);
	free_candev(net);

	sysfs_remove_group(&spi->dev.kobj, &ch9431_attribute_group);
	sysfs_remove_link(NULL, "ch9431");

	dev_info(&spi->dev, "ch9431 CAN device driver removed\n");
}
#else
static int ch9431_drv_remove(struct spi_device *spi)
{
	struct ch9431_priv *ch9431 = spi_get_drvdata(spi);
	struct net_device *net = ch9431->ndev;

	unregister_candev(net);

	ch9431_power_enable(ch9431->power, 0);

	destroy_workqueue(ch9431->wq);
	ch9431->wq = NULL;

	free_irq(ch9431->spi->irq, ch9431);
	free_candev(net);

	sysfs_remove_group(&spi->dev.kobj, &ch9431_attribute_group);
	sysfs_remove_link(NULL, "ch9431");

	dev_info(&spi->dev, "ch9431 CAN device driver removed\n");

	return 0;
}
#endif

static const struct of_device_id ch9431_match_table[] = {
	{ .compatible = "wch,ch9431" },
	{}
};

MODULE_DEVICE_TABLE(of, ch9431_match_table);

static const struct spi_device_id ch9431_id_table[] = {
	{
		.name = "ch9431",
		.driver_data = (kernel_ulong_t)0x9431,
	},
	{}
};
MODULE_DEVICE_TABLE(spi, ch9431_id_table);

static struct spi_driver ch9431_driver = {
	.driver = {
		.name = DRVNAME_CH9431,
		.of_match_table = ch9431_match_table,
	},
	.id_table = ch9431_id_table,
	.probe = ch9431_probe,
	.remove = ch9431_drv_remove,
};
module_spi_driver(ch9431_driver);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(VERSION_DESC);
MODULE_LICENSE("GPL");
