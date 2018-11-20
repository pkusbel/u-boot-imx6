/*
 * (C) Copyright 2008 - 2009
 * Windriver, <www.windriver.com>
 * Tom Rix <Tom.Rix@windriver.com>
 *
 * Copyright 2011 Sebastian Andrzej Siewior <bigeasy@linutronix.de>
 *
 * Copyright 2014 Linaro, Ltd.
 * Rob Herring <robh@kernel.org>
 *
 * Copyright (C) 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <config.h>
#include <common.h>
#include <errno.h>
#include <stdlib.h>
#include <fastboot.h>
#include <malloc.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/composite.h>
#include <linux/compiler.h>
#include <version.h>
#include <g_dnl.h>
#include <environment.h>
#ifdef CONFIG_FASTBOOT_FLASH_MMC_DEV
#include <fb_mmc.h>
#endif

#ifdef CONFIG_IMX_TRUSTY_OS
extern int armv7_init_nonsec(void);
extern void trusty_os_init(void);
#include <trusty/libtipc.h>
#endif

#ifdef CONFIG_FSL_FASTBOOT
#include <asm/mach-imx/sys_proto.h>
#include <fsl_fastboot.h>
#include <mmc.h>
#include <android_image.h>
#include <asm/bootm.h>
#include <nand.h>
#include <part.h>
#include <sparse_format.h>
#include <image-sparse.h>
#include <image.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/setup.h>
#ifdef CONFIG_ANDROID_RECOVERY
#include <recovery.h>
#endif
#endif

#ifdef CONFIG_BCB_SUPPORT
#include "bcb.h"
#endif

#ifdef CONFIG_AVB_SUPPORT
#include <fsl_avb.h>
#endif

#define FASTBOOT_VERSION		"0.4"

#ifdef CONFIG_FASTBOOT_LOCK
#include "fastboot_lock_unlock.h"
#endif
#define FASTBOOT_COMMON_VAR_NUM	13
#define FASTBOOT_VAR_YES    "yes"
#define FASTBOOT_VAR_NO     "no"

#define ANDROID_GPT_OFFSET         0
#define ANDROID_GPT_SIZE           0x100000
#define ANDROID_GPT_END	           0x4400
#define FASTBOOT_INTERFACE_CLASS	0xff
#define FASTBOOT_INTERFACE_SUB_CLASS	0x42
#define FASTBOOT_INTERFACE_PROTOCOL	0x03

#define RX_ENDPOINT_MAXIMUM_PACKET_SIZE_2_0  (0x0200)
#define RX_ENDPOINT_MAXIMUM_PACKET_SIZE_1_1  (0x0040)
#define TX_ENDPOINT_MAXIMUM_PACKET_SIZE      (0x0040)

#define EP_BUFFER_SIZE			4096

#if defined (CONFIG_ARCH_MX8) || defined (CONFIG_ARCH_MX8M)
#define DST_DECOMPRESS_LEN 1024*1024*32
#endif

/*
 * EP_BUFFER_SIZE must always be an integral multiple of maxpacket size
 * (64 or 512 or 1024), else we break on certain controllers like DWC3
 * that expect bulk OUT requests to be divisible by maxpacket size.
 */
#ifdef CONFIG_SYSTEM_RAMDISK_SUPPORT
static bool is_recovery_mode;
#endif

/* common variables of fastboot getvar command */
char *fastboot_common_var[FASTBOOT_COMMON_VAR_NUM] = {
	"version",
	"version-bootloader",
	"version-baseband",
	"product",
	"secure",
	"max-download-size",
	"erase-block-size",
	"logical-block-size",
	"unlocked",
	"off-mode-charge",
	"battery-voltage",
	"variant",
	"battery-soc-ok"
};

/* Boot metric variables */
boot_metric metrics = {
	.bll_1 = 0,
	.ble_1 = 0,
	.kl    = 0,
	.kd    = 0,
	.avb   = 0,
	.odt   = 0,
	.sw    = 0
};

#ifdef CONFIG_USB_GADGET
typedef struct usb_req usb_req;
struct usb_req {
	struct usb_request *in_req;
	usb_req *next;
};

struct f_fastboot {
	struct usb_function usb_function;

	/* IN/OUT EP's and corresponding requests */
	struct usb_ep *in_ep, *out_ep;
	struct usb_request *in_req, *out_req;
	usb_req *front, *rear;
};

static inline struct f_fastboot *func_to_fastboot(struct usb_function *f)
{
	return container_of(f, struct f_fastboot, usb_function);
}
static int strcmp_l1(const char *s1, const char *s2);

static struct f_fastboot *fastboot_func;
static struct usb_endpoint_descriptor fs_ep_in = {
	.bLength            = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType    = USB_DT_ENDPOINT,
	.bEndpointAddress   = USB_DIR_IN,
	.bmAttributes       = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize     = cpu_to_le16(64),
};

static struct usb_endpoint_descriptor fs_ep_out = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_OUT,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= cpu_to_le16(64),
};

static struct usb_endpoint_descriptor hs_ep_in = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= cpu_to_le16(512),
};

static struct usb_endpoint_descriptor hs_ep_out = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_OUT,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= cpu_to_le16(512),
};

static struct usb_interface_descriptor interface_desc = {
	.bLength		= USB_DT_INTERFACE_SIZE,
	.bDescriptorType	= USB_DT_INTERFACE,
	.bInterfaceNumber	= 0x00,
	.bAlternateSetting	= 0x00,
	.bNumEndpoints		= 0x02,
	.bInterfaceClass	= FASTBOOT_INTERFACE_CLASS,
	.bInterfaceSubClass	= FASTBOOT_INTERFACE_SUB_CLASS,
	.bInterfaceProtocol	= FASTBOOT_INTERFACE_PROTOCOL,
};

static struct usb_descriptor_header *fb_fs_function[] = {
	(struct usb_descriptor_header *)&interface_desc,
	(struct usb_descriptor_header *)&fs_ep_in,
	(struct usb_descriptor_header *)&fs_ep_out,
};

static struct usb_descriptor_header *fb_hs_function[] = {
	(struct usb_descriptor_header *)&interface_desc,
	(struct usb_descriptor_header *)&hs_ep_in,
	(struct usb_descriptor_header *)&hs_ep_out,
	NULL,
};

static struct usb_endpoint_descriptor *
fb_ep_desc(struct usb_gadget *g, struct usb_endpoint_descriptor *fs,
	    struct usb_endpoint_descriptor *hs)
{
	if (gadget_is_dualspeed(g) && g->speed == USB_SPEED_HIGH)
		return hs;
	return fs;
}
#endif

/*
 * static strings, in UTF-8
 */
static const char fastboot_name[] = "Android Fastboot";

static struct usb_string fastboot_string_defs[] = {
	[0].s = fastboot_name,
	{  }			/* end of list */
};

static struct usb_gadget_strings stringtab_fastboot = {
	.language	= 0x0409,	/* en-us */
	.strings	= fastboot_string_defs,
};

static struct usb_gadget_strings *fastboot_strings[] = {
	&stringtab_fastboot,
	NULL,
};

#ifdef CONFIG_FSL_FASTBOOT

#ifndef TRUSTY_OS_MMC_BLKS
#define TRUSTY_OS_MMC_BLKS 0x7FF
#endif
#ifndef TEE_HWPARTITION_ID
#define TEE_HWPARTITION_ID 2
#endif

#define ANDROID_MBR_OFFSET	    0
#define ANDROID_MBR_SIZE	    0x200
#ifdef  CONFIG_BOOTLOADER_OFFSET_33K
#define ANDROID_BOOTLOADER_OFFSET   0x8400
#else
#define ANDROID_BOOTLOADER_OFFSET   0x400
#endif
#define ANDROID_BOOTLOADER_SIZE	    0x1FFC00

#define MMC_SATA_BLOCK_SIZE 512
#define FASTBOOT_FBPARTS_ENV_MAX_LEN 1024
/* To support the Android-style naming of flash */
#define MAX_PTN		    32
struct fastboot_ptentry g_ptable[MAX_PTN];
unsigned int g_pcount;
struct fastboot_device_info fastboot_devinfo;


enum {
	PTN_GPT_INDEX = 0,
	PTN_TEE_INDEX,
	PTN_BOOTLOADER_INDEX,
};

static struct cmd_fastboot_interface interface = {
	.rx_handler            = NULL,
	.reset_handler         = NULL,
	.product_name          = NULL,
	.serial_no             = NULL,
	.nand_block_size       = 0,
	.transfer_buffer       = (unsigned char *)0xffffffff,
	.transfer_buffer_size  = 0,
};

#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
/*judge wether the gpt image and bootloader image are overlay*/
bool bootloader_gpt_overlay(void)
{
	return (g_ptable[PTN_GPT_INDEX].partition_id  == g_ptable[PTN_BOOTLOADER_INDEX].partition_id  &&
		ANDROID_BOOTLOADER_OFFSET < ANDROID_GPT_END);
}

int write_backup_gpt(void)
{
	int mmc_no = 0;
	struct mmc *mmc;
	struct blk_desc *dev_desc;

	mmc_no = fastboot_devinfo.dev_id;
	mmc = find_mmc_device(mmc_no);
	if (mmc == NULL) {
		printf("invalid mmc device\n");
		return -1;
	}
	dev_desc = blk_get_dev("mmc", mmc_no);

	/* write backup get partition */
	if (write_backup_gpt_partitions(dev_desc, interface.transfer_buffer)) {
		printf("writing GPT image fail\n");
		return -1;
	}

	printf("flash backup gpt image successfully\n");
	return 0;
}
#endif

#if defined(CONFIG_FASTBOOT_STORAGE_SATA)
static void process_erase_sata(const char *cmdbuf, char *response)
{
    return;
}
#endif

static void parameters_setup(void)
{
	interface.nand_block_size = 0;
	interface.transfer_buffer =
				(unsigned char *)CONFIG_FASTBOOT_BUF_ADDR;
	interface.transfer_buffer_size =
				CONFIG_FASTBOOT_BUF_SIZE;
}

static int _fastboot_setup_dev(void)
{
	char *fastboot_env;
	fastboot_env = env_get("fastboot_dev");

	if (fastboot_env) {
		if (!strcmp(fastboot_env, "sata")) {
			fastboot_devinfo.type = DEV_SATA;
			fastboot_devinfo.dev_id = 0;
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
		} else if (!strncmp(fastboot_env, "mmc", 3)) {
			fastboot_devinfo.type = DEV_MMC;
			fastboot_devinfo.dev_id = mmc_get_env_dev();
#endif
		}
	} else {
		return 1;
	}

	return 0;
}

#if defined(CONFIG_FASTBOOT_STORAGE_SATA) \
	|| defined(CONFIG_FASTBOOT_STORAGE_MMC)
/**
   @mmc_dos_partition_index: the partition index in mbr.
   @mmc_partition_index: the boot partition or user partition index,
   not related to the partition table.
 */
static int _fastboot_parts_add_ptable_entry(int ptable_index,
				      int mmc_dos_partition_index,
				      int mmc_partition_index,
				      const char *name,
				      const char *fstype,
				      struct blk_desc *dev_desc,
				      struct fastboot_ptentry *ptable)
{
	disk_partition_t info;

	if (part_get_info(dev_desc,
			       mmc_dos_partition_index, &info)) {
		debug("Bad partition index:%d for partition:%s\n",
		       mmc_dos_partition_index, name);
		return -1;
	}
	ptable[ptable_index].start = info.start;
	ptable[ptable_index].length = info.size;
	ptable[ptable_index].partition_id = mmc_partition_index;
	ptable[ptable_index].partition_index = mmc_dos_partition_index;
	strcpy(ptable[ptable_index].name, (const char *)info.name);

#ifdef CONFIG_PARTITION_UUIDS
	strcpy(ptable[ptable_index].uuid, (const char *)info.uuid);
#endif
#ifdef CONFIG_ANDROID_AB_SUPPORT
	if (!strcmp((const char *)info.name, FASTBOOT_PARTITION_SYSTEM_A) ||
		!strcmp((const char *)info.name, FASTBOOT_PARTITION_SYSTEM_B) ||
		!strcmp((const char *)info.name, FASTBOOT_PARTITION_DATA))
#else
	if (!strcmp((const char *)info.name, FASTBOOT_PARTITION_SYSTEM) ||
		!strcmp((const char *)info.name, FASTBOOT_PARTITION_DATA) ||
		!strcmp((const char *)info.name, FASTBOOT_PARTITION_DEVICE) ||
		!strcmp((const char *)info.name, FASTBOOT_PARTITION_CACHE))
#endif
		strcpy(ptable[ptable_index].fstype, "ext4");
	else
		strcpy(ptable[ptable_index].fstype, "raw");
	return 0;
}

static int _fastboot_parts_load_from_ptable(void)
{
	int i;
#ifdef CONFIG_CMD_SATA
	int sata_device_no;
#endif

	/* mmc boot partition: -1 means no partition, 0 user part., 1 boot part.
	 * default is no partition, for emmc default user part, except emmc*/
	int boot_partition = FASTBOOT_MMC_NONE_PARTITION_ID;
	int user_partition = FASTBOOT_MMC_NONE_PARTITION_ID;

	struct mmc *mmc;
	struct blk_desc *dev_desc;
	struct fastboot_ptentry ptable[MAX_PTN];

	/* sata case in env */
	if (fastboot_devinfo.type == DEV_SATA) {
#ifdef CONFIG_CMD_SATA
		puts("flash target is SATA\n");
		if (sata_initialize())
			return -1;
		sata_device_no = CONFIG_FASTBOOT_SATA_NO;
		if (sata_device_no >= CONFIG_SYS_SATA_MAX_DEVICE) {
			printf("Unknown SATA(%d) device for fastboot\n",
				sata_device_no);
			return -1;
		}
		dev_desc = sata_get_dev(sata_device_no);
#else /*! CONFIG_CMD_SATA*/
		puts("SATA isn't buildin\n");
		return -1;
#endif /*! CONFIG_CMD_SATA*/
	} else if (fastboot_devinfo.type == DEV_MMC) {
		int mmc_no = 0;
		mmc_no = fastboot_devinfo.dev_id;

		printf("flash target is MMC:%d\n", mmc_no);
		mmc = find_mmc_device(mmc_no);
		if (mmc && mmc_init(mmc))
			printf("MMC card init failed!\n");

		dev_desc = blk_get_dev("mmc", mmc_no);
		if (!dev_desc || dev_desc->type == DEV_TYPE_UNKNOWN) {
			printf("** Block device MMC %d not supported\n",
				mmc_no);
			return -1;
		}

		/* multiple boot paritions for eMMC 4.3 later */
		if (mmc->part_config != MMCPART_NOAVAILABLE) {
			boot_partition = FASTBOOT_MMC_BOOT_PARTITION_ID;
			user_partition = FASTBOOT_MMC_USER_PARTITION_ID;
		}
	} else {
		printf("Can't setup partition table on this device %d\n",
			fastboot_devinfo.type);
		return -1;
	}

	memset((char *)ptable, 0,
		    sizeof(struct fastboot_ptentry) * (MAX_PTN));
	/* GPT */
	strcpy(ptable[PTN_GPT_INDEX].name, FASTBOOT_PARTITION_GPT);
	ptable[PTN_GPT_INDEX].start = ANDROID_GPT_OFFSET / dev_desc->blksz;
	ptable[PTN_GPT_INDEX].length = ANDROID_GPT_SIZE  / dev_desc->blksz;
	ptable[PTN_GPT_INDEX].partition_id = user_partition;
	ptable[PTN_GPT_INDEX].flags = FASTBOOT_PTENTRY_FLAGS_UNERASEABLE;
	strcpy(ptable[PTN_GPT_INDEX].fstype, "raw");

	/* Trusty OS */
	strcpy(ptable[PTN_TEE_INDEX].name, FASTBOOT_PARTITION_TEE);
	ptable[PTN_TEE_INDEX].start = 0;
	ptable[PTN_TEE_INDEX].length = TRUSTY_OS_MMC_BLKS;
	ptable[PTN_TEE_INDEX].partition_id = TEE_HWPARTITION_ID;
	strcpy(ptable[PTN_TEE_INDEX].fstype, "raw");

	/* Bootloader */
	strcpy(ptable[PTN_BOOTLOADER_INDEX].name, FASTBOOT_PARTITION_BOOTLOADER);
	ptable[PTN_BOOTLOADER_INDEX].start =
				ANDROID_BOOTLOADER_OFFSET / dev_desc->blksz;
	ptable[PTN_BOOTLOADER_INDEX].length =
				 ANDROID_BOOTLOADER_SIZE / dev_desc->blksz;
	ptable[PTN_BOOTLOADER_INDEX].partition_id = boot_partition;
	ptable[PTN_BOOTLOADER_INDEX].flags = FASTBOOT_PTENTRY_FLAGS_UNERASEABLE;
	strcpy(ptable[PTN_BOOTLOADER_INDEX].fstype, "raw");

	int tbl_idx;
	int part_idx = 1;
	int ret;
	for (tbl_idx = PTN_BOOTLOADER_INDEX + 1; tbl_idx < MAX_PTN; tbl_idx++) {
		ret = _fastboot_parts_add_ptable_entry(tbl_idx,
				part_idx++,
				user_partition,
				NULL,
				NULL,
				dev_desc, ptable);
		if (ret)
			break;
	}
	for (i = 0; i <= part_idx; i++)
		fastboot_flash_add_ptn(&ptable[i]);

	return 0;
}
#endif /*CONFIG_FASTBOOT_STORAGE_SATA || CONFIG_FASTBOOT_STORAGE_MMC*/

static void _fastboot_load_partitions(void)
{
	g_pcount = 0;
#if defined(CONFIG_FASTBOOT_STORAGE_SATA) \
	|| defined(CONFIG_FASTBOOT_STORAGE_MMC)
	_fastboot_parts_load_from_ptable();
#endif
}

/*
 * Android style flash utilties */
void fastboot_flash_add_ptn(struct fastboot_ptentry *ptn)
{
	if (g_pcount < MAX_PTN) {
		memcpy(g_ptable + g_pcount, ptn, sizeof(struct fastboot_ptentry));
		g_pcount++;
	}
}

void fastboot_flash_dump_ptn(void)
{
	unsigned int n;
	for (n = 0; n < g_pcount; n++) {
		struct fastboot_ptentry *ptn = g_ptable + n;
		printf("idx %d, ptn %d name='%s' start=%d len=%d\n",
			n, ptn->partition_index, ptn->name, ptn->start, ptn->length);
	}
}


struct fastboot_ptentry *fastboot_flash_find_ptn(const char *name)
{
	unsigned int n;

	for (n = 0; n < g_pcount; n++) {
		/* Make sure a substring is not accepted */
		if (strlen(name) == strlen(g_ptable[n].name)) {
			if (0 == strcmp(g_ptable[n].name, name))
				return g_ptable + n;
		}
	}

	printf("can't find partition: %s, dump the partition table\n", name);
	fastboot_flash_dump_ptn();
	return 0;
}

int fastboot_flash_find_index(const char *name)
{
	struct fastboot_ptentry *ptentry = fastboot_flash_find_ptn(name);
	if (ptentry == NULL) {
		printf("cannot get the partion info for %s\n",name);
		return -1;
	}
	return ptentry->partition_index;
}

struct fastboot_ptentry *fastboot_flash_get_ptn(unsigned int n)
{
	if (n < g_pcount)
		return g_ptable + n;
	else
		return 0;
}

unsigned int fastboot_flash_get_ptn_count(void)
{
	return g_pcount;
}

#ifdef CONFIG_FSL_FASTBOOT
void board_fastboot_setup(void)
{
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	static char boot_dev_part[32];
	u32 dev_no;
#endif
	switch (get_boot_device()) {
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	case SD1_BOOT:
	case SD2_BOOT:
	case SD3_BOOT:
	case SD4_BOOT:
	case MMC1_BOOT:
	case MMC2_BOOT:
	case MMC3_BOOT:
	case MMC4_BOOT:
		dev_no = mmc_get_env_dev();
		sprintf(boot_dev_part,"mmc%d",dev_no);
		if (!env_get("fastboot_dev"))
			env_set("fastboot_dev", boot_dev_part);
		sprintf(boot_dev_part, "boota mmc%d", dev_no);
		if (!env_get("bootcmd"))
			env_set("bootcmd", boot_dev_part);
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_MMC*/
	default:
		printf("unsupported boot devices %d\n", get_boot_device());
		break;
	}

	/* add soc type into bootargs */
	if (is_mx6dqp()) {
		if (!env_get("soc_type"))
			env_set("soc_type", "imx6qp");
	} else if (is_mx6dq()) {
		if (!env_get("soc_type"))
			env_set("soc_type", "imx6q");
	} else if (is_mx6sdl()) {
		if (!env_get("soc_type"))
			env_set("soc_type", "imx6dl");
	} else if (is_mx6sx()) {
		if (!env_get("soc_type"))
			env_set("soc_type", "imx6sx");
	} else if (is_mx6sl()) {
		if (!env_get("soc_type"))
			env_set("soc_type", "imx6sl");
	} else if (is_mx6ul()) {
		if (!env_get("soc_type"))
			env_set("soc_type", "imx6ul");
	} else if (is_mx7()) {
		if (!env_get("soc_type"))
			env_set("soc_type", "imx7d");
	} else if (is_mx7ulp()) {
		if (!env_get("soc_type"))
			env_set("soc_type", "imx7ulp");
	} else if (is_mx8qm()) {
		if (!env_get("soc_type"))
			env_set("soc_type", "imx8qm");
	} else if (is_mx8qxp()) {
		if (!env_get("soc_type"))
			env_set("soc_type", "imx8qxp");
	} else if (is_mx8m()) {
		if (!env_get("soc_type"))
			env_set("soc_type", "imx8mq");
	}
}

#ifdef CONFIG_ANDROID_RECOVERY
void board_recovery_setup(void)
{
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	static char boot_dev_part[32];
	u32 dev_no;
#endif
	int bootdev = get_boot_device();
	switch (bootdev) {
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	case SD1_BOOT:
	case SD2_BOOT:
	case SD3_BOOT:
	case SD4_BOOT:
	case MMC1_BOOT:
	case MMC2_BOOT:
	case MMC3_BOOT:
	case MMC4_BOOT:
		dev_no = mmc_get_env_dev();
		sprintf(boot_dev_part,"boota mmc%d recovery",dev_no);
		if (!env_get("bootcmd_android_recovery"))
			env_set("bootcmd_android_recovery", boot_dev_part);
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_MMC*/
	default:
		printf("Unsupported bootup device for recovery: dev: %d\n",
			bootdev);
		return;
	}

	printf("setup env for recovery..\n");
	env_set("bootcmd", "run bootcmd_android_recovery");
}
#endif /*CONFIG_ANDROID_RECOVERY*/
#endif /*CONFIG_FSL_FASTBOOT*/

#if defined(CONFIG_AVB_SUPPORT) && defined(CONFIG_MMC)
static AvbABOps fsl_avb_ab_ops = {
	.read_ab_metadata = fsl_read_ab_metadata,
	.write_ab_metadata = fsl_write_ab_metadata,
	.ops = NULL
};
#ifdef CONFIG_AVB_ATX
static AvbAtxOps fsl_avb_atx_ops = {
	.ops = NULL,
	.read_permanent_attributes = fsl_read_permanent_attributes,
	.read_permanent_attributes_hash = fsl_read_permanent_attributes_hash
};
#endif
static AvbOps fsl_avb_ops = {
	.ab_ops = &fsl_avb_ab_ops,
#ifdef CONFIG_AVB_ATX
	.atx_ops = &fsl_avb_atx_ops,
#endif
	.read_from_partition = fsl_read_from_partition_multi,
	.write_to_partition = fsl_write_to_partition,
#ifdef CONFIG_AVB_ATX
	.validate_vbmeta_public_key = avb_atx_validate_vbmeta_public_key,
#else
	.validate_vbmeta_public_key = fsl_validate_vbmeta_public_key_rpmb,
#endif
	.read_rollback_index = fsl_read_rollback_index_rpmb,
	.write_rollback_index = fsl_write_rollback_index_rpmb,
	.read_is_device_unlocked = fsl_read_is_device_unlocked,
	.get_unique_guid_for_partition = fsl_get_unique_guid_for_partition,
	.get_size_of_partition = fsl_get_size_of_partition
};
#endif

#ifdef CONFIG_IMX_TRUSTY_OS
void tee_setup(void)
{
	/* load tee from boot1 of eMMC. */
	int mmcc = mmc_get_env_dev();
	struct blk_desc *dev_desc = NULL;

	struct mmc *mmc;
	mmc = find_mmc_device(mmcc);
	if (!mmc) {
	            printf("boota: cannot find '%d' mmc device\n", mmcc);
		            goto fail;
	}

	dev_desc = blk_get_dev("mmc", mmcc);
	if (NULL == dev_desc) {
	            printf("** Block device MMC %d not supported\n", mmcc);
		            goto fail;
	}

	/* below was i.MX mmc operation code */
	if (mmc_init(mmc)) {
	            printf("mmc%d init failed\n", mmcc);
		            goto fail;
	}

	struct fastboot_ptentry *tee_pte;
	char *tee_ptn = FASTBOOT_PARTITION_TEE;
	tee_pte = fastboot_flash_find_ptn(tee_ptn);
	mmc_switch_part(mmc, TEE_HWPARTITION_ID);
	if (!tee_pte) {
		printf("boota: cannot find tee partition!\n");
	}

	if (mmc->block_dev.block_read(dev_desc, tee_pte->start,
		    tee_pte->length, (void *)TRUSTY_OS_ENTRY) < 0) {
		printf("Failed to load tee.");
	}
	mmc_switch_part(mmc, FASTBOOT_MMC_USER_PARTITION_ID);

#ifdef NON_SECURE_FASTBOOT
	armv7_init_nonsec();
	trusty_os_init();
	trusty_ipc_init();
#endif

fail:
	return;

}
#endif

void fastboot_setup(void)
{
#ifdef CONFIG_USB_GADGET
	struct tag_serialnr serialnr;
	char serial[17];

	get_board_serial(&serialnr);
	sprintf(serial, "%08x%08x", serialnr.high, serialnr.low);
	g_dnl_set_serialnumber(serial);
#endif
	/*execute board relevant initilizations for preparing fastboot */
	board_fastboot_setup();

	/*get the fastboot dev*/
	_fastboot_setup_dev();


	/*load partitions information for the fastboot dev*/
	_fastboot_load_partitions();

	parameters_setup();
#ifdef CONFIG_AVB_SUPPORT
	fsl_avb_ab_ops.ops = &fsl_avb_ops;
#ifdef CONFIG_AVB_ATX
	fsl_avb_atx_ops.ops = &fsl_avb_ops;
#endif
#endif
}

/* Get the Boot mode from BCB cmd or Key pressed */
static FbBootMode fastboot_get_bootmode(void)
{
	int ret = 0;
	int boot_mode = BOOTMODE_NORMAL;
	char command[32];
#ifdef CONFIG_ANDROID_RECOVERY
	if(is_recovery_key_pressing()) {
		boot_mode = BOOTMODE_RECOVERY_KEY_PRESSED;
		return boot_mode;
	}
#endif
	ret = bcb_read_command(command);
	if (ret < 0) {
		printf("read command failed\n");
		return boot_mode;
	}
	if (!strcmp(command, FASTBOOT_BCB_CMD)) {
		boot_mode = BOOTMODE_FASTBOOT_BCB_CMD;
	}
#ifdef CONFIG_ANDROID_RECOVERY
	else if (!strcmp(command, RECOVERY_BCB_CMD)) {
		boot_mode = BOOTMODE_RECOVERY_BCB_CMD;
	}
#endif

	/* Clean the mode once its read out,
	   no matter what in the mode string */
	memset(command, 0, 32);
	bcb_write_command(command);
	return boot_mode;
}

#ifdef CONFIG_SYSTEM_RAMDISK_SUPPORT
/* Setup booargs for taking the system parition as ramdisk */
static void fastboot_setup_system_boot_args(const char *slot, bool append_root)
{
	const char *system_part_name = NULL;
	if(slot == NULL)
		return;
	if(!strncmp(slot, "_a", strlen("_a")) || !strncmp(slot, "boot_a", strlen("boot_a"))) {
		system_part_name = FASTBOOT_PARTITION_SYSTEM_A;
	}
	else if(!strncmp(slot, "_b", strlen("_b")) || !strncmp(slot, "boot_b", strlen("boot_b"))) {
		system_part_name = FASTBOOT_PARTITION_SYSTEM_B;
	}
	struct fastboot_ptentry *ptentry = fastboot_flash_find_ptn(system_part_name);
	if(ptentry != NULL) {
		char bootargs_3rd[ANDR_BOOT_ARGS_SIZE];
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
		if (append_root) {
			u32 dev_no = mmc_map_to_kernel_blk(mmc_get_env_dev());
			sprintf(bootargs_3rd, "skip_initramfs root=/dev/mmcblk%dp%d",
					dev_no,
					ptentry->partition_index);
		} else {
			sprintf(bootargs_3rd, "skip_initramfs");
		}
		env_set("bootargs_3rd", bootargs_3rd);
#endif
	}
}
#endif
/* export to lib_arm/board.c */
void fastboot_run_bootmode(void)
{
	FbBootMode boot_mode = fastboot_get_bootmode();
	switch(boot_mode){
	case BOOTMODE_FASTBOOT_BCB_CMD:
		/* Make the boot into fastboot mode*/
		puts("Fastboot: Got bootloader commands!\n");
#ifdef CONFIG_SYSTEM_RAMDISK_SUPPORT
		is_recovery_mode = false;
#endif
		run_command("fastboot 0", 0);
		break;
#ifdef CONFIG_ANDROID_RECOVERY
	case BOOTMODE_RECOVERY_BCB_CMD:
	case BOOTMODE_RECOVERY_KEY_PRESSED:
		/* Make the boot into recovery mode */
		puts("Fastboot: Got Recovery key pressing or recovery commands!\n");
#ifdef CONFIG_SYSTEM_RAMDISK_SUPPORT
		is_recovery_mode = true;
#else
		board_recovery_setup();
#endif
		break;
#endif
	default:
		/* skip special mode boot*/
#ifdef CONFIG_SYSTEM_RAMDISK_SUPPORT
		is_recovery_mode = false;
#endif
		puts("Fastboot: Normal\n");
		break;
	}
}

#ifdef CONFIG_CMD_BOOTA
  /* Section for Android bootimage format support
   * Refer:
   * http://android.git.kernel.org/?p=platform/system/core.git;a=blob;
   * f=mkbootimg/bootimg.h
   */

void
bootimg_print_image_hdr(struct andr_img_hdr *hdr)
{
#ifdef DEBUG
	int i;
	printf("   Image magic:   %s\n", hdr->magic);

	printf("   kernel_size:   0x%x\n", hdr->kernel_size);
	printf("   kernel_addr:   0x%x\n", hdr->kernel_addr);

	printf("   rdisk_size:   0x%x\n", hdr->ramdisk_size);
	printf("   rdisk_addr:   0x%x\n", hdr->ramdisk_addr);

	printf("   second_size:   0x%x\n", hdr->second_size);
	printf("   second_addr:   0x%x\n", hdr->second_addr);

	printf("   tags_addr:   0x%x\n", hdr->tags_addr);
	printf("   page_size:   0x%x\n", hdr->page_size);

	printf("   name:      %s\n", hdr->name);
	printf("   cmdline:   %s\n", hdr->cmdline);

	for (i = 0; i < 8; i++)
		printf("   id[%d]:   0x%x\n", i, hdr->id[i]);
#endif
}

static struct andr_img_hdr boothdr __aligned(ARCH_DMA_MINALIGN);

#if defined(CONFIG_AVB_SUPPORT) && defined(CONFIG_MMC)
/* we can use avb to verify Trusty if we want */
const char *requested_partitions[] = {"boot", 0};

int do_boota(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]) {

	struct andr_img_hdr *hdr = NULL;
	void *boot_buf = NULL;
	u32 avb_metric;
	ulong addr = 0;
	ulong image_size;
	bool check_image_arm64 =  false;

#if defined (CONFIG_ARCH_MX8) || defined (CONFIG_ARCH_MX8M)
	size_t lz4_len = DST_DECOMPRESS_LEN;
#endif

	AvbABFlowResult avb_result;
	AvbSlotVerifyData *avb_out_data;
	AvbPartitionData *avb_loadpart;

#ifdef CONFIG_FASTBOOT_LOCK
	/* check lock state */
	FbLockState lock_status = fastboot_get_lock_stat();
	if (lock_status == FASTBOOT_LOCK_ERROR) {
		printf("In boota get fastboot lock status error. Set lock status\n");
		fastboot_set_lock_stat(FASTBOOT_LOCK);
		lock_status = FASTBOOT_LOCK;
	}
	bool allow_fail = (lock_status == FASTBOOT_UNLOCK ? true : false);
	avb_metric = get_timer(0);
	/* if in lock state, do avb verify */
	avb_result = avb_ab_flow_fast(&fsl_avb_ab_ops, requested_partitions, allow_fail,
			AVB_HASHTREE_ERROR_MODE_RESTART_AND_INVALIDATE, &avb_out_data);
	/* get the duration of avb */
	metrics.avb = get_timer(avb_metric);

	if ((avb_result == AVB_AB_FLOW_RESULT_OK) ||
			(avb_result == AVB_AB_FLOW_RESULT_OK_WITH_VERIFICATION_ERROR)) {
		assert(avb_out_data != NULL);
		/* load the first partition */
		avb_loadpart = avb_out_data->loaded_partitions;
		assert(avb_loadpart != NULL);
		/* we should use avb_part_data->data as boot image */
		/* boot image is already read by avb */
		hdr = (struct andr_img_hdr *)avb_loadpart->data;

		if (android_image_check_header(hdr)) {
			printf("boota: bad boot image magic\n");
			goto fail;
		}
		if (avb_result == AVB_AB_FLOW_RESULT_OK)
			printf(" verify OK, boot '%s%s'\n",
					avb_loadpart->partition_name, avb_out_data->ab_suffix);
		else {
			printf(" verify FAIL, state: UNLOCK\n");
			printf(" boot '%s%s' still\n",
					avb_loadpart->partition_name, avb_out_data->ab_suffix);
		}

		/* @19labs/nabil: fix buffer overflow in snprintf */
		char bootargs_sec[2048];
		int ret;
		if (lock_status == FASTBOOT_LOCK) {
			ret = snprintf(bootargs_sec,
				       sizeof(bootargs_sec),
				       "androidboot.verifiedbootstate=green androidboot.slot_suffix=%s %s",
				       avb_out_data->ab_suffix, avb_out_data->cmdline);
		} else {
			ret = snprintf(bootargs_sec,
				       sizeof(bootargs_sec),
				       "androidboot.verifiedbootstate=orange androidboot.slot_suffix=%s %s",
				       avb_out_data->ab_suffix, avb_out_data->cmdline);
		}
		if ((ret < 0 || ret >= sizeof(bootargs_sec))) {
			printf("boota: cmdline too large for bootargs_sec\n");
			goto fail;
		}
		/* @19labs/nabil - end */

		env_set("bootargs_sec", bootargs_sec);
#ifdef CONFIG_SYSTEM_RAMDISK_SUPPORT
		if(!is_recovery_mode) {
			if(avb_out_data->cmdline != NULL && strstr(avb_out_data->cmdline, "root="))
				fastboot_setup_system_boot_args(avb_out_data->ab_suffix, false);
			else
				fastboot_setup_system_boot_args(avb_out_data->ab_suffix, true);
		}
#endif
		image_size = avb_loadpart->data_size;
#if defined (CONFIG_ARCH_MX8) || defined (CONFIG_ARCH_MX8M)
		/* If we are using uncompressed kernel image, copy it directly to
		 * hdr->kernel_addr, if we are using compressed lz4 kernel image,
		 * we need to decompress the kernel image first. */
		if (image_arm64((void *)((ulong)hdr + hdr->page_size))) {
			memcpy((void *)(long)hdr->kernel_addr,
					(void *)((ulong)hdr + hdr->page_size), hdr->kernel_size);
		} else {
#ifdef CONFIG_LZ4
			if (ulz4fn((void *)((ulong)hdr + hdr->page_size),
						hdr->kernel_size, (void *)(ulong)hdr->kernel_addr, &lz4_len) != 0) {
				printf("Decompress kernel fail!\n");
				goto fail;
			}
#else /* CONFIG_LZ4 */
			printf("please enable CONFIG_LZ4 if we're using compressed lz4 kernel image!\n");
			goto fail;
#endif /* CONFIG_LZ4 */
		}
#else /* CONFIG_ARCH_MX8 || CONFIG_ARCH_MX8M */
		/* copy kernel image and boot header to hdr->kernel_addr - hdr->page_size */
		memcpy((void *)(ulong)(hdr->kernel_addr - hdr->page_size), (void *)hdr,
				hdr->page_size + ALIGN(hdr->kernel_size, hdr->page_size));
#endif /* CONFIG_ARCH_MX8 || CONFIG_ARCH_MX8M */
	} else if (lock_status == FASTBOOT_LOCK) { /* && verify fail */
		/* if in lock state, verify enforce fail */
		printf(" verify FAIL, state: LOCK\n");
		goto fail;
	} else { /* lock_status == FASTBOOT_UNLOCK && get unacceptable verify fail */
		/* if in unlock state, log the verify state */
		printf(" verify FAIL, state: UNLOCK\n");
#endif
		/* if lock/unlock not enabled or verify fail
		 * in unlock state, will try boot */
		size_t num_read;
		hdr = &boothdr;

		char bootimg[8];
		char *slot = select_slot(&fsl_avb_ab_ops);
		if (slot == NULL) {
			printf("boota: no bootable slot\n");
			goto fail;
		}
		sprintf(bootimg, "boot%s", slot);
		printf(" boot '%s' still\n", bootimg);
		/* maybe we should use bootctl to select a/b
		 * but libavb doesn't export a/b select */
		if (fsl_avb_ops.read_from_partition(&fsl_avb_ops, bootimg,
					0, sizeof(boothdr), hdr, &num_read) != AVB_IO_RESULT_OK &&
				num_read != sizeof(boothdr)) {
			printf("boota: read bootimage head error\n");
			goto fail;
		}

		if (android_image_check_header(hdr)) {
		  printf("boota: bad boot image magic\n");
		  goto fail;
		}
		image_size = android_image_get_end(hdr) - (ulong)hdr;
#if defined (CONFIG_ARCH_MX8) || defined (CONFIG_ARCH_MX8M)
		boot_buf = malloc(image_size);
		/* Load boot image */
		if (fsl_avb_ops.read_from_partition(&fsl_avb_ops, bootimg,
				0, image_size, boot_buf, &num_read) != AVB_IO_RESULT_OK
				&& num_read != image_size) {
			printf("boota: read boot image error\n");
			goto fail;
		}
		/* If we are using uncompressed kernel image, copy it directly to
		 * hdr->kernel_addr, if we are using compressed lz4 kernel image,
		 * we need to decompress the kernel image first. */
		if (image_arm64((void *)((ulong)boot_buf + hdr->page_size))) {
			memcpy((void *)(ulong)hdr->kernel_addr,
					(void *)((ulong)boot_buf + hdr->page_size), hdr->kernel_size);
		} else {
#ifdef CONFIG_LZ4
			if (ulz4fn((void *)((ulong)boot_buf + hdr->page_size),
						hdr->kernel_size, (void *)(ulong)hdr->kernel_addr, &lz4_len) != 0) {
				printf("Decompress kernel fail!\n");
				goto fail;
			}
#else /* CONFIG_LZ4 */
			printf("please enable CONFIG_LZ4 if we're using compressed lz4 kernel image!\n");
			goto fail;
#endif /* CONFIG_LZ4 */
		}
		hdr = (struct andr_img_hdr *)boot_buf;
#else /* CONFIG_ARCH_MX8 || CONFIG_ARCH_MX8M */
		if (fsl_avb_ops.read_from_partition(&fsl_avb_ops, bootimg,
				0, image_size, (void *)(ulong)(hdr->kernel_addr - hdr->page_size), &num_read) != AVB_IO_RESULT_OK
				&& num_read != image_size) {
			printf("boota: read boot image error\n");
			goto fail;
		}
		hdr = (struct andr_img_hdr *)(ulong)(hdr->kernel_addr - hdr->page_size);
#endif /* CONFIG_ARCH_MX8 || CONFIG_ARCH_MX8M */

		char bootargs_sec[ANDR_BOOT_ARGS_SIZE];
		sprintf(bootargs_sec,
				"androidboot.verifiedbootstate=orange androidboot.slot_suffix=%s", slot);
		env_set("bootargs_sec", bootargs_sec);
#ifdef CONFIG_SYSTEM_RAMDISK_SUPPORT
		if(!is_recovery_mode)
			fastboot_setup_system_boot_args(slot, true);
#endif
#ifdef CONFIG_FASTBOOT_LOCK
	}
#endif

	flush_cache((ulong)load_addr, image_size);
	check_image_arm64  = image_arm64((void *)(ulong)hdr->kernel_addr);
#ifdef CONFIG_SYSTEM_RAMDISK_SUPPORT
	if (is_recovery_mode)
		memcpy((void *)(ulong)hdr->ramdisk_addr, (void *)(ulong)hdr + hdr->page_size
				+ ALIGN(hdr->kernel_size, hdr->page_size), hdr->ramdisk_size);
#else
	memcpy((void *)(ulong)hdr->ramdisk_addr, (void *)(ulong)hdr + hdr->page_size
			+ ALIGN(hdr->kernel_size, hdr->page_size), hdr->ramdisk_size);
#endif
#ifdef CONFIG_OF_LIBFDT
	/* load the dtb file */
	if (hdr->second_size && hdr->second_addr) {
		memcpy((void *)(ulong)hdr->second_addr, (void *)(ulong)hdr + hdr->page_size
			+ ALIGN(hdr->kernel_size, hdr->page_size)
			+ ALIGN(hdr->ramdisk_size, hdr->page_size), hdr->second_size);
	}
#endif /*CONFIG_OF_LIBFDT*/
	if (check_image_arm64) {
		android_image_get_kernel(hdr, 0, NULL, NULL);
		addr = hdr->kernel_addr;
	} else {
		addr = (ulong)(hdr->kernel_addr - hdr->page_size);
	}
	printf("kernel   @ %08x (%d)\n", hdr->kernel_addr, hdr->kernel_size);
	printf("ramdisk  @ %08x (%d)\n", hdr->ramdisk_addr, hdr->ramdisk_size);
#ifdef CONFIG_OF_LIBFDT
	if (hdr->second_size)
		printf("fdt      @ %08x (%d)\n", hdr->second_addr, hdr->second_size);
#endif /*CONFIG_OF_LIBFDT*/

	char boot_addr_start[12];
	char ramdisk_addr[25];
	char fdt_addr[12];

	char *boot_args[] = { NULL, boot_addr_start, ramdisk_addr, fdt_addr};
	if (check_image_arm64)
		boot_args[0] = "booti";
	else
		boot_args[0] = "bootm";

	sprintf(boot_addr_start, "0x%lx", addr);
	sprintf(ramdisk_addr, "0x%x:0x%x", hdr->ramdisk_addr, hdr->ramdisk_size);
	sprintf(fdt_addr, "0x%x", hdr->second_addr);

/* no need to pass ramdisk addr for normal boot mode when enable CONFIG_SYSTEM_RAMDISK_SUPPORT*/
#ifdef CONFIG_SYSTEM_RAMDISK_SUPPORT
	if (!is_recovery_mode)
		boot_args[2] = NULL;
#endif
	if (avb_out_data != NULL)
		avb_slot_verify_data_free(avb_out_data);
	if (boot_buf != NULL)
		free(boot_buf);

#ifdef CONFIG_IMX_TRUSTY_OS
	/* put ql-tipc to release resource for Linux */
	trusty_ipc_shutdown();
#endif
	if (check_image_arm64) {
#ifdef CONFIG_CMD_BOOTI
		do_booti(NULL, 0, 4, boot_args);
#else
		debug("please enable CONFIG_CMD_BOOTI when kernel are Image");
#endif
	} else {
		do_bootm(NULL, 0, 4, boot_args);
	}

	/* This only happens if image is somehow faulty so we start over */
	do_reset(NULL, 0, 0, NULL);

	return 1;
fail:
	/* avb has no recovery */
	if (avb_out_data != NULL)
		avb_slot_verify_data_free(avb_out_data);
	if (boot_buf != NULL)
		free(boot_buf);

	return run_command("fastboot 0", 0);
}

U_BOOT_CMD(
	boota,	2,	1,	do_boota,
	"boota   - boot android bootimg \n",
	"boot from current mmc with avb verify\n"
);
#else /* CONFIG_AVB_SUPPORT */
/* boota <addr> [ mmc0 | mmc1 [ <partition> ] ] */
int do_boota(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	ulong addr = 0;
	char *ptn = "boot";
	int mmcc = -1;
	struct andr_img_hdr *hdr = &boothdr;
	ulong image_size;
	bool check_image_arm64 =  false;
	int i = 0;

	for (i = 0; i < argc; i++)
		printf("%s ", argv[i]);
	printf("\n");

	if (argc < 2)
		return -1;

	mmcc = simple_strtoul(argv[1]+3, NULL, 10);

	if (argc > 2)
		ptn = argv[2];

	if (mmcc != -1) {
#ifdef CONFIG_MMC
		struct fastboot_ptentry *pte;
		struct mmc *mmc;
		disk_partition_t info;
		struct blk_desc *dev_desc = NULL;
		unsigned bootimg_sectors;

		memset((void *)&info, 0 , sizeof(disk_partition_t));
		/* i.MX use MBR as partition table, so this will have
		   to find the start block and length for the
		   partition name and register the fastboot pte we
		   define the partition number of each partition in
		   config file
		 */
		mmc = find_mmc_device(mmcc);
		if (!mmc) {
			printf("boota: cannot find '%d' mmc device\n", mmcc);
			goto fail;
		}
		dev_desc = blk_get_dev("mmc", mmcc);
		if (!dev_desc || dev_desc->type == DEV_TYPE_UNKNOWN) {
			printf("** Block device MMC %d not supported\n", mmcc);
			goto fail;
		}

		/* below was i.MX mmc operation code */
		if (mmc_init(mmc)) {
			printf("mmc%d init failed\n", mmcc);
			goto fail;
		}

		pte = fastboot_flash_find_ptn(ptn);
		if (!pte) {
			printf("boota: cannot find '%s' partition\n", ptn);
			goto fail;
		}

		if (mmc->block_dev.block_read(dev_desc, pte->start,
					      1, (void *)hdr) < 0) {
			printf("boota: mmc failed to read bootimg header\n");
			goto fail;
		}

		if (android_image_check_header(hdr)) {
			printf("boota: bad boot image magic\n");
			goto fail;
		}

		image_size = android_image_get_end(hdr) - (ulong)hdr;
		bootimg_sectors = image_size/512;

		if (mmc->block_dev.block_read(dev_desc,	pte->start,
					bootimg_sectors,
					(void *)(hdr->kernel_addr - hdr->page_size)) < 0) {
			printf("boota: mmc failed to read bootimage\n");
			goto fail;
		}
		check_image_arm64  = image_arm64((void *)hdr->kernel_addr);
#ifdef CONFIG_FASTBOOT_LOCK
		int verifyresult = -1;
#endif

#ifdef CONFIG_FASTBOOT_LOCK
		int lock_status = fastboot_get_lock_stat();
		if (lock_status == FASTBOOT_LOCK_ERROR) {
			printf("In boota get fastboot lock status error. Set lock status\n");
			fastboot_set_lock_stat(FASTBOOT_LOCK);
		}
		display_lock(fastboot_get_lock_stat(), verifyresult);
#endif
		/* load the ramdisk file */
		memcpy((void *)hdr->ramdisk_addr, (void *)hdr->kernel_addr
			+ ALIGN(hdr->kernel_size, hdr->page_size), hdr->ramdisk_size);

#ifdef CONFIG_OF_LIBFDT
		/* load the dtb file */
		if (hdr->second_size && hdr->second_addr) {
			memcpy((void *)hdr->second_addr, (void *)hdr->kernel_addr
				+ ALIGN(hdr->kernel_size, hdr->page_size)
				+ ALIGN(hdr->ramdisk_size, hdr->page_size), hdr->second_size);
		}
#endif /*CONFIG_OF_LIBFDT*/

#else /*! CONFIG_MMC*/
		return -1;
#endif /*! CONFIG_MMC*/
	} else {
		printf("boota: parameters is invalid. only support mmcX device\n");
		return -1;
	}

	printf("kernel   @ %08x (%d)\n", hdr->kernel_addr, hdr->kernel_size);
	printf("ramdisk  @ %08x (%d)\n", hdr->ramdisk_addr, hdr->ramdisk_size);
#ifdef CONFIG_OF_LIBFDT
	if (hdr->second_size)
		printf("fdt      @ %08x (%d)\n", hdr->second_addr, hdr->second_size);
#endif /*CONFIG_OF_LIBFDT*/

	char boot_addr_start[12];
	char ramdisk_addr[25];
	char fdt_addr[12];
	char *boot_args[] = { NULL, boot_addr_start, ramdisk_addr, fdt_addr};
	if (check_image_arm64 ) {
		addr = hdr->kernel_addr;
		boot_args[0] = "booti";
	} else {
		addr = hdr->kernel_addr - hdr->page_size;
		boot_args[0] = "bootm";
	}

	sprintf(boot_addr_start, "0x%lx", addr);
	sprintf(ramdisk_addr, "0x%x:0x%x", hdr->ramdisk_addr, hdr->ramdisk_size);
	sprintf(fdt_addr, "0x%x", hdr->second_addr);
	if (check_image_arm64) {
		android_image_get_kernel(hdr, 0, NULL, NULL);
#ifdef CONFIG_CMD_BOOTI
		do_booti(NULL, 0, 4, boot_args);
#else
		debug("please enable CONFIG_CMD_BOOTI when kernel are Image");
#endif
	} else {
		do_bootm(NULL, 0, 4, boot_args);
	}
	/* This only happens if image is somehow faulty so we start over */
	do_reset(NULL, 0, 0, NULL);

	return 1;

fail:
#if defined(CONFIG_FSL_FASTBOOT)
	return run_command("fastboot 0", 0);
#else /*! CONFIG_FSL_FASTBOOT*/
	return -1;
#endif /*! CONFIG_FSL_FASTBOOT*/
}

U_BOOT_CMD(
	boota,	3,	1,	do_boota,
	"boota   - boot android bootimg from memory\n",
	"[<addr> | mmc0 | mmc1 | mmc2 | mmcX] [<partition>]\n    "
	"- boot application image stored in memory or mmc\n"
	"\t'addr' should be the address of boot image "
	"which is zImage+ramdisk.img\n"
	"\t'mmcX' is the mmc device you store your boot.img, "
	"which will read the boot.img from 1M offset('/boot' partition)\n"
	"\t 'partition' (optional) is the partition id of your device, "
	"if no partition give, will going to 'boot' partition\n"
);
#endif /* CONFIG_AVB_SUPPORT */
#endif	/* CONFIG_CMD_BOOTA */
#endif

#ifdef CONFIG_USB_GADGET
static void rx_handler_command(struct usb_ep *ep, struct usb_request *req);

static void fastboot_complete(struct usb_ep *ep, struct usb_request *req)
{
	int status = req->status;
	if (!status)
		return;
	printf("status: %d ep '%s' trans: %d\n", status, ep->name, req->actual);
}

static int fastboot_bind(struct usb_configuration *c, struct usb_function *f)
{
	int id;
	struct usb_gadget *gadget = c->cdev->gadget;
	struct f_fastboot *f_fb = func_to_fastboot(f);
	const char *s;

	/* DYNAMIC interface numbers assignments */
	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	interface_desc.bInterfaceNumber = id;

	id = usb_string_id(c->cdev);
	if (id < 0)
		return id;
	fastboot_string_defs[0].id = id;
	interface_desc.iInterface = id;

	f_fb->in_ep = usb_ep_autoconfig(gadget, &fs_ep_in);
	if (!f_fb->in_ep)
		return -ENODEV;
	f_fb->in_ep->driver_data = c->cdev;

	f_fb->out_ep = usb_ep_autoconfig(gadget, &fs_ep_out);
	if (!f_fb->out_ep)
		return -ENODEV;
	f_fb->out_ep->driver_data = c->cdev;

	f->descriptors = fb_fs_function;

	if (gadget_is_dualspeed(gadget)) {
		/* Assume endpoint addresses are the same for both speeds */
		hs_ep_in.bEndpointAddress = fs_ep_in.bEndpointAddress;
		hs_ep_out.bEndpointAddress = fs_ep_out.bEndpointAddress;
		/* copy HS descriptors */
		f->hs_descriptors = fb_hs_function;
	}

	s = env_get("serial#");
	if (s)
		g_dnl_set_serialnumber((char *)s);

	return 0;
}

static void fastboot_unbind(struct usb_configuration *c, struct usb_function *f)
{
	memset(fastboot_func, 0, sizeof(*fastboot_func));
}

static void fastboot_disable(struct usb_function *f)
{
	usb_req *req;
	struct f_fastboot *f_fb = func_to_fastboot(f);

	usb_ep_disable(f_fb->out_ep);
	usb_ep_disable(f_fb->in_ep);

	if (f_fb->out_req) {
		free(f_fb->out_req->buf);
		usb_ep_free_request(f_fb->out_ep, f_fb->out_req);
		f_fb->out_req = NULL;
	}
	if (f_fb->in_req) {
		free(f_fb->in_req->buf);
		usb_ep_free_request(f_fb->in_ep, f_fb->in_req);

		/* disable usb request FIFO */
		while(f_fb->front != NULL) {
			req = f_fb->front;
			f_fb->front = f_fb->front->next;
			free(req->in_req->buf);
			usb_ep_free_request(f_fb->in_ep, req->in_req);
			free(req);
		}

		f_fb->rear = NULL;
		f_fb->in_req = NULL;
	}
}

static struct usb_request *fastboot_start_ep(struct usb_ep *ep)
{
	struct usb_request *req;

	req = usb_ep_alloc_request(ep, 0);
	if (!req)
		return NULL;

	req->length = EP_BUFFER_SIZE;
	req->buf = memalign(CONFIG_SYS_CACHELINE_SIZE, EP_BUFFER_SIZE);
	if (!req->buf) {
		usb_ep_free_request(ep, req);
		return NULL;
	}

	memset(req->buf, 0, req->length);
	return req;
}

static int fastboot_set_alt(struct usb_function *f,
			    unsigned interface, unsigned alt)
{
	int ret;
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_gadget *gadget = cdev->gadget;
	struct f_fastboot *f_fb = func_to_fastboot(f);
	const struct usb_endpoint_descriptor *d;

	debug("%s: func: %s intf: %d alt: %d\n",
	      __func__, f->name, interface, alt);

	d = fb_ep_desc(gadget, &fs_ep_out, &hs_ep_out);
	ret = usb_ep_enable(f_fb->out_ep, d);
	if (ret) {
		puts("failed to enable out ep\n");
		return ret;
	}

	f_fb->out_req = fastboot_start_ep(f_fb->out_ep);
	if (!f_fb->out_req) {
		puts("failed to alloc out req\n");
		ret = -EINVAL;
		goto err;
	}
	f_fb->out_req->complete = rx_handler_command;

	d = fb_ep_desc(gadget, &fs_ep_in, &hs_ep_in);
	ret = usb_ep_enable(f_fb->in_ep, d);
	if (ret) {
		puts("failed to enable in ep\n");
		goto err;
	}

	f_fb->in_req = fastboot_start_ep(f_fb->in_ep);
	if (!f_fb->in_req) {
		puts("failed alloc req in\n");
		ret = -EINVAL;
		goto err;
	}
	f_fb->in_req->complete = fastboot_complete;

	ret = usb_ep_queue(f_fb->out_ep, f_fb->out_req, 0);
	if (ret)
		goto err;

	return 0;
err:
	fastboot_disable(f);
	return ret;
}

static int fastboot_add(struct usb_configuration *c)
{
	struct f_fastboot *f_fb = fastboot_func;
	int status;

	debug("%s: cdev: 0x%p\n", __func__, c->cdev);

	if (!f_fb) {
		f_fb = memalign(CONFIG_SYS_CACHELINE_SIZE, sizeof(*f_fb));
		if (!f_fb)
			return -ENOMEM;

		fastboot_func = f_fb;
		memset(f_fb, 0, sizeof(*f_fb));
	}

	f_fb->usb_function.name = "f_fastboot";
	f_fb->usb_function.bind = fastboot_bind;
	f_fb->usb_function.unbind = fastboot_unbind;
	f_fb->usb_function.set_alt = fastboot_set_alt;
	f_fb->usb_function.disable = fastboot_disable;
	f_fb->usb_function.strings = fastboot_strings;

	status = usb_add_function(c, &f_fb->usb_function);
	if (status) {
		free(f_fb);
		fastboot_func = f_fb;
	}

	return status;
}
DECLARE_GADGET_BIND_CALLBACK(usb_dnl_fastboot, fastboot_add);

static int fastboot_tx_write(const char *buffer, unsigned int buffer_size)
{
	struct usb_request *in_req = fastboot_func->in_req;
	int ret;

	memcpy(in_req->buf, buffer, buffer_size);
	in_req->length = buffer_size;

	usb_ep_dequeue(fastboot_func->in_ep, in_req);

	ret = usb_ep_queue(fastboot_func->in_ep, in_req, 0);
	if (ret)
		printf("Error %d on queue\n", ret);
	return 0;
}

int fastboot_tx_write_str(const char *buffer)
{
	return fastboot_tx_write(buffer, strlen(buffer));
}

static void compl_do_reset(struct usb_ep *ep, struct usb_request *req)
{
	do_reset(NULL, 0, 0, NULL);
}

int __weak fb_set_reboot_flag(void)
{
	return -ENOSYS;
}

static int strcmp_l1(const char *s1, const char *s2)
{
	if (!s1 || !s2)
		return -1;
	return strncmp(s1, s2, strlen(s1));
}

bool is_slotvar(char *cmd)
{
	assert(cmd != NULL);
	if (!strcmp_l1("has-slot:", cmd) ||
		!strcmp_l1("slot-successful:", cmd) ||
		!strcmp_l1("slot-count", cmd) ||
		!strcmp_l1("slot-suffixes", cmd) ||
		!strcmp_l1("current-slot", cmd) ||
		!strcmp_l1("slot-unbootable:", cmd) ||
		!strcmp_l1("slot-retry-count:", cmd))
			return true;
	return false;
}

#if !defined(PRODUCT_NAME)
#define PRODUCT_NAME "NXP i.MX"
#endif

#if !defined(VARIANT_NAME)
#define VARIANT_NAME "NXP i.MX"
#endif

static unsigned int rx_bytes_expected(struct usb_ep *ep)
{
	int rx_remain = fastboot_data_remaining();
	unsigned int rem;
	unsigned int maxpacket = ep->maxpacket;

	if (rx_remain <= 0)
		return 0;
	else if (rx_remain > EP_BUFFER_SIZE)
		return EP_BUFFER_SIZE;

	/*
	 * Some controllers e.g. DWC3 don't like OUT transfers to be
	 * not ending in maxpacket boundary. So just make them happy by
	 * always requesting for integral multiple of maxpackets.
	 * This shouldn't bother controllers that don't care about it.
	 */
	rem = rx_remain % maxpacket;
	if (rem > 0)
		rx_remain = rx_remain + (maxpacket - rem);

	return rx_remain;
}

static void rx_handler_dl_image(struct usb_ep *ep, struct usb_request *req)
{
	char response[FASTBOOT_RESPONSE_LEN] = {0};
	unsigned int transfer_size = fastboot_data_remaining();
	const unsigned char *buffer = req->buf;
	unsigned int buffer_size = req->actual;

	if (req->status != 0) {
		printf("Bad status: %d\n", req->status);
		return;
	}

	if (buffer_size < transfer_size)
		transfer_size = buffer_size;

	fastboot_data_download(buffer, transfer_size, response);
	if (response[0]) {
		fastboot_tx_write_str(response);
	} else if (!fastboot_data_remaining()) {
		fastboot_data_complete(response);

		/*
		 * Reset global transfer variable
		 */
		req->complete = rx_handler_command;
		req->length = EP_BUFFER_SIZE;

		fastboot_tx_write_str(response);
	} else {
		req->length = rx_bytes_expected(ep);
	}

	req->actual = 0;
	usb_ep_queue(ep, req, 0);
}

static void do_exit_on_complete(struct usb_ep *ep, struct usb_request *req)
{
	g_dnl_trigger_detach();
}

static void do_bootm_on_complete(struct usb_ep *ep, struct usb_request *req)
{
	fastboot_boot();
	do_exit_on_complete(ep, req);
}

static void rx_handler_command(struct usb_ep *ep, struct usb_request *req)
{
	char *cmdbuf = req->buf;
	char response[FASTBOOT_RESPONSE_LEN] = {0};
	int cmd = -1;

	if (req->status != 0 || req->length == 0)
		return;

	if (req->actual < req->length) {
		cmdbuf[req->actual] = '\0';
		cmd = fastboot_handle_command(cmdbuf, response);
	} else {
		pr_err("buffer overflow");
		fastboot_fail("buffer overflow", response);
	}

	if (!strncmp("DATA", response, 4)) {
		req->complete = rx_handler_dl_image;
		req->length = rx_bytes_expected(ep);
	}

	fastboot_tx_write_str(response);

	if (!strncmp("OKAY", response, 4)) {
		switch (cmd) {
		case FASTBOOT_COMMAND_BOOT:
			fastboot_func->in_req->complete = do_bootm_on_complete;
			break;

		case FASTBOOT_COMMAND_CONTINUE:
			fastboot_func->in_req->complete = do_exit_on_complete;
			break;

		case FASTBOOT_COMMAND_REBOOT:
		case FASTBOOT_COMMAND_REBOOT_BOOTLOADER:
			fastboot_func->in_req->complete = compl_do_reset;
			break;
		}
	}

	*cmdbuf = '\0';
	req->actual = 0;
	usb_ep_queue(ep, req, 0);
}
#endif
