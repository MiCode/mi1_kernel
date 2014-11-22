/* linux/arch/arm/mach-msm/board-xiaomi-mmc.c
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/sdio_func.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>
#include <linux/if.h>

#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/mach/mmc.h>
#include <mach/board.h>

#include <mach/vreg.h>

#include "board-xiaomi.h"
#include "devices.h"

#undef XIAOMI_DEBUG_MMC

int bcm_wifi_power(int on);
int bcm_wifi_reset(int on);
int bcm_wifi_set_carddetect(int on);

#define PREALLOC_WLAN_NUMBER_OF_SECTIONS        4
#define PREALLOC_WLAN_NUMBER_OF_BUFFERS         160
#define PREALLOC_WLAN_SECTION_HEADER            24

#define WLAN_SECTION_SIZE_0     (PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_1     (PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_2     (PREALLOC_WLAN_NUMBER_OF_BUFFERS * 512)
#define WLAN_SECTION_SIZE_3     (PREALLOC_WLAN_NUMBER_OF_BUFFERS * 1024)

#define WLAN_SKB_BUF_NUM        16

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

typedef struct wifi_mem_prealloc_struct {
	void *mem_ptr;
	unsigned long size;
} wifi_mem_prealloc_t;

static wifi_mem_prealloc_t wifi_mem_array[PREALLOC_WLAN_NUMBER_OF_SECTIONS] = {
	{NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER)}
};

static void *xiaomi_wifi_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_NUMBER_OF_SECTIONS)
		return wlan_static_skb;
	if ((section < 0) || (section > PREALLOC_WLAN_NUMBER_OF_SECTIONS))
		return NULL;
	if (wifi_mem_array[section].size < size)
		return NULL;
	return wifi_mem_array[section].mem_ptr;
}

int __init xiaomi_init_wifi_mem(void)
{
	int i;

	for (i = 0; (i < WLAN_SKB_BUF_NUM); i++) {
		if (i < (WLAN_SKB_BUF_NUM / 2))
			wlan_static_skb[i] = dev_alloc_skb(4096);
		else
			wlan_static_skb[i] = dev_alloc_skb(8192);
	}
	for (i = 0; (i < PREALLOC_WLAN_NUMBER_OF_SECTIONS); i++) {
		wifi_mem_array[i].mem_ptr = kmalloc(wifi_mem_array[i].size,
						    GFP_KERNEL);
		if (wifi_mem_array[i].mem_ptr == NULL)
			return -ENOMEM;
	}
	return 0;
}

static struct resource xiaomi_wifi_resources[] = {
	[0] = {
	       .name = "bcm4329_wlan_irq",
	       .start = MSM_GPIO_TO_INT(XIAOMI_GPIO_WIFI_IRQ),
	       .end = MSM_GPIO_TO_INT(XIAOMI_GPIO_WIFI_IRQ),
	       .flags =
	       IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL |
	       IORESOURCE_IRQ_SHAREABLE,
	       },
};

unsigned short device_id;
static unsigned char mac_addr[IFHWADDRLEN] = { 0, 0x90, 0x4c, 0, 0, 0 };

static int wifi_get_mac_addr(unsigned char *buf)
{
	uint rand_mac;
	if (device_id != 0x4330)
		return -EINVAL;

	if (!buf)
		return -EFAULT;

	if ((mac_addr[4] == 0) && (mac_addr[5] == 0)) {
		srandom32((uint) jiffies);
		rand_mac = random32();
		mac_addr[3] = (unsigned char)rand_mac;
		mac_addr[4] = (unsigned char)(rand_mac >> 8);
		mac_addr[5] = (unsigned char)(rand_mac >> 16);
	}
	memcpy(buf, mac_addr, IFHWADDRLEN);
	return 0;
}

/* Customized Locale table : OPTIONAL feature */
#define WLC_CNTRY_BUF_SZ        4
typedef struct cntry_locales_custom {
	char iso_abbrev[WLC_CNTRY_BUF_SZ];
	char custom_locale[WLC_CNTRY_BUF_SZ];
	int custom_locale_rev;
} cntry_locales_custom_t;

static cntry_locales_custom_t brcm_wlan_translate_custom_table[] = {
	/* Table should be filled out based on custom platform regulatory requirement */

	{"", "XY", 4},		/* Universal if Country code is unknown or empty */
	{"US", "US", 69},	/* input ISO "US" to : US regrev 69 */
	{"CA", "US", 69},	/* input ISO "CA" to : US regrev 69 */
	{"EU", "EU", 5},	/* European union countries to : EU regrev 05 */
	{"AT", "EU", 5},
	{"BE", "EU", 5},
	{"BG", "EU", 5},
	{"CY", "EU", 5},
	{"CZ", "EU", 5},
	{"DK", "EU", 5},
	{"EE", "EU", 5},
	{"FI", "EU", 5},
	{"FR", "EU", 5},
	{"DE", "EU", 5},
	{"GR", "EU", 5},
	{"HU", "EU", 5},
	{"IE", "EU", 5},
	{"IT", "EU", 5},
	{"LV", "EU", 5},
	{"LI", "EU", 5},
	{"LT", "EU", 5},
	{"LU", "EU", 5},
	{"MT", "EU", 5},
	{"NL", "EU", 5},
	{"PL", "EU", 5},
	{"PT", "EU", 5},
	{"RO", "EU", 5},
	{"SK", "EU", 5},
	{"SI", "EU", 5},
	{"ES", "EU", 5},
	{"SE", "EU", 5},
	{"GB", "EU", 5},
	{"KR", "XY", 3},
	{"AU", "XY", 3},
	{"CN", "XY", 3},	/* input ISO "CN" to : XY regrev 03 */
	{"TW", "XY", 3},
	{"AR", "XY", 3},
	{"MX", "XY", 3},
	{"IL", "IL", 0},
	{"CH", "CH", 0},
	{"TR", "TR", 0},
	{"NO", "NO", 0},
};

static void *brcm_wlan_get_country_code(char *ccode)
{
	int size = ARRAY_SIZE(brcm_wlan_translate_custom_table);
	int i;

	if (!ccode)
		return NULL;
	printk("wlan: brcm set ccode\n");
	for (i = 0; i < size; i++)
		if (strcmp
		    (ccode,
		     brcm_wlan_translate_custom_table[i].iso_abbrev) == 0)
			return &brcm_wlan_translate_custom_table[i];
	return &brcm_wlan_translate_custom_table[0];
}

static struct wifi_platform_data xiaomi_wifi_control = {
	.set_power = bcm_wifi_power,
	.set_reset = bcm_wifi_reset,
	.set_carddetect = bcm_wifi_set_carddetect,
	.mem_prealloc = xiaomi_wifi_mem_prealloc,
	.get_mac_addr = wifi_get_mac_addr,
	.get_country_code = brcm_wlan_get_country_code,
};

static struct platform_device xiaomi_wifi_device = {
	.name = "bcm4329_wlan",
	.id = 1,
	.num_resources = ARRAY_SIZE(xiaomi_wifi_resources),
	.resource = xiaomi_wifi_resources,
	.dev = {
		.platform_data = &xiaomi_wifi_control,
		},
};

#if !defined(SDIO_VENDOR_ID_BROADCOM)
#define SDIO_VENDOR_ID_BROADCOM		0x02d0
#endif /* !defined(SDIO_VENDOR_ID_BROADCOM) */

#define SDIO_DEVICE_ID_BROADCOM_DEFAULT	0x0000

#if !defined(SDIO_DEVICE_ID_BROADCOM_4325_SDGWB)
#define SDIO_DEVICE_ID_BROADCOM_4325_SDGWB	0x0492	/* BCM94325SDGWB */
#endif /* !defined(SDIO_DEVICE_ID_BROADCOM_4325_SDGWB) */
#if !defined(SDIO_DEVICE_ID_BROADCOM_4325)
#define SDIO_DEVICE_ID_BROADCOM_4325	0x0493
#endif /* !defined(SDIO_DEVICE_ID_BROADCOM_4325) */
#if !defined(SDIO_DEVICE_ID_BROADCOM_4329)
#define SDIO_DEVICE_ID_BROADCOM_4329	0x4329
#endif /* !defined(SDIO_DEVICE_ID_BROADCOM_4329) */
#if !defined(SDIO_DEVICE_ID_BROADCOM_4319)
#define SDIO_DEVICE_ID_BROADCOM_4319	0x4319
#endif /* !defined(SDIO_DEVICE_ID_BROADCOM_4319) */
/* devices we support, null terminated */
static const struct sdio_device_id bcmsdh_sdmmc_ids[] = {
	{SDIO_DEVICE(SDIO_VENDOR_ID_BROADCOM, SDIO_DEVICE_ID_BROADCOM_DEFAULT)},
	{SDIO_DEVICE
	 (SDIO_VENDOR_ID_BROADCOM, SDIO_DEVICE_ID_BROADCOM_4325_SDGWB)},
	{SDIO_DEVICE(SDIO_VENDOR_ID_BROADCOM, SDIO_DEVICE_ID_BROADCOM_4325)},
	{SDIO_DEVICE(SDIO_VENDOR_ID_BROADCOM, SDIO_DEVICE_ID_BROADCOM_4329)},
	{SDIO_DEVICE(SDIO_VENDOR_ID_BROADCOM, SDIO_DEVICE_ID_BROADCOM_4319)},
	{SDIO_DEVICE_CLASS(SDIO_CLASS_NONE)},
	{ /* end: all zeroes */ },
};

static struct kobject *wifi_module;
struct kobj_attribute wifi_attr;

static ssize_t modalias_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	return sprintf(buf, "wifi_module=%x\n", device_id);
}

static void fake_bcm4329_cleanup(struct work_struct *work);
static struct delayed_work cleanup_work;

static int fake_bcm4329_probe(struct sdio_func *func,
			      const struct sdio_device_id *id)
{
	int error = 0;
	printk("fake func->num %d\n", func->num);
	if (func->num == 2) {
		device_id = func->device;
		wifi_module = kobject_create_and_add("wifi_properties", NULL);
		if (wifi_module == NULL) {
			printk("create wifi_module failed\n");
			return 0;
		}

		sysfs_attr_init(&wifi_attr.attr);
		wifi_attr.attr.name = "wifi_module";
		wifi_attr.attr.mode = (S_IRUSR | S_IRGRP | S_IROTH);
		wifi_attr.show = modalias_show;

		error = sysfs_create_file(wifi_module, &wifi_attr.attr);
		if (error)
			printk("create wifi_module failed\n");
		schedule_delayed_work(&cleanup_work, 0);
	}
	return 0;
}

static void fake_bcm4329_remove(struct sdio_func *func)
{
}

static struct sdio_driver fake_bcm4329_driver = {
	.probe = fake_bcm4329_probe,
	.remove = fake_bcm4329_remove,
	.id_table = bcmsdh_sdmmc_ids,
};

static void fake_bcm4329_cleanup(struct work_struct *work)
{
	sdio_unregister_driver(&fake_bcm4329_driver);
	xiaomi_wifi_control.set_power(0);
	xiaomi_wifi_control.set_carddetect(0);
}

static int __init xiaomi_wifi_init(void)
{
	int ret;

	ret = gpio_request(XIAOMI_GPIO_WIFI_IRQ, "wifi_irq");
	if (ret) {
		printk("wifi irq gpio request error, %s\n", __func__);
		goto err_gpio_wifiirq;
	}
	gpio_direction_input(XIAOMI_GPIO_WIFI_IRQ);
	ret = gpio_request(XIAOMI_GPIO_WIFI_SHUTDOWN_N, "wifi_shutdown");
	if (ret) {
		printk("wifi shutdown gpio request error, %s\n", __func__);
		goto err_gpio_wifishutdown;
	}
	gpio_direction_output(XIAOMI_GPIO_WIFI_SHUTDOWN_N, 0);
	xiaomi_init_wifi_mem();
	INIT_DELAYED_WORK(&cleanup_work, fake_bcm4329_cleanup);
	xiaomi_wifi_control.set_power(1);
	xiaomi_wifi_control.set_carddetect(1);
	ret = platform_device_register(&xiaomi_wifi_device);
	if (!ret)
		ret = sdio_register_driver(&fake_bcm4329_driver);
	return ret;
err_gpio_wifiirq:
	gpio_free(XIAOMI_GPIO_WIFI_IRQ);
err_gpio_wifishutdown:
	gpio_free(XIAOMI_GPIO_WIFI_SHUTDOWN_N);
	return ret;
}

late_initcall(xiaomi_wifi_init);
