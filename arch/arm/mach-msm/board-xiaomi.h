/* arch/arm/mach-msm/board-XIAOMI.h
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/

#ifndef __ARCH_ARM_MACH_MSM_BOARD_XIAOMI_H
#define __ARCH_ARM_MACH_MSM_BOARD_XIAOMI_H


#include <mach/board.h>

#define XIAOMI_GPIO_PMIC_SLEEPCLOCK     37
#define XIAOMI_GPIO_PMIC_AUDIO_PA_SD1	12

#define XIAOMI_GPIO_WIFI_SHUTDOWN_N	149
#define XIAOMI_GPIO_WIFI_IRQ		125 /* WL_HOST_WAKE */


#define XIAOMI_GPIO_BT_UART1_TX		53
#define XIAOMI_GPIO_BT_UART1_RX		54
#define XIAOMI_GPIO_BT_UART1_CTS	55
#define XIAOMI_GPIO_BT_UART1_RTS	56

#define XIAOMI_GPIO_BT_RESET_N		154
#define XIAOMI_GPIO_BT_SHUTDOWN_N	147

#define XIAOMI_GPIO_BT_WAKE		140
#define XIAOMI_GPIO_BT_HOST_WAKE	126

/* Bluetooth PCM */
#define XIAOMI_BT_PCM_OUT		111
#define XIAOMI_BT_PCM_IN		112
#define XIAOMI_BT_PCM_SYNC		113
#define XIAOMI_BT_PCM_CLK		114

#define XIAOMI_GPIO_HS_UART_SWITCH_EN	145
#define XIAOMI_MMC_MSM_CARD_HW_DET_PIN	127

#endif /* __ARCH_ARM_MACH_MSM_BOARD_XIAOMI_H */
