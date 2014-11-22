/*
 * Copyright (c) 2011 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#ifndef _RMI_DRIVER_H
#define _RMI_DRIVER_H

#define RMI_DRIVER_VERSION "1.3"

#define RMI_PRODUCT_ID_LENGTH    10
#define RMI_PRODUCT_INFO_LENGTH   2
#define RMI_DATE_CODE_LENGTH      3

union f01_device_status {
	struct {
		u8 status_code:4;
		u8 reserved:2;
		u8 flash_prog:1;
		u8 unconfigured:1;
	};
	u8 reg;
};

struct rmi_driver_data {
	struct rmi_function_container rmi_functions;

	struct rmi_function_container *f01_container;
	bool f01_bootloader_mode;

	int num_of_irq_regs;
	int irq_count;
	u8 *current_irq_mask;
	u8 *irq_mask_store;
	bool irq_stored;
	struct mutex irq_mutex;
	struct mutex pdt_mutex;

	unsigned char pdt_props;
	unsigned char bsr;
	bool enabled;

#ifdef CONFIG_PM
	bool suspended;
	struct mutex suspend_mutex;

	void *pm_data;
	int (*pre_suspend) (const void *pm_data);
	int (*post_resume) (const void *pm_data);
#endif

#ifdef CONFIG_RMI4_DEBUG
#ifdef CONFIG_RMI4_SPI
	struct dentry *debugfs_delay;
#endif
	struct dentry *debugfs_phys;
#endif

	void *data;
};

struct pdt_entry {
	u8 query_base_addr:8;
	u8 command_base_addr:8;
	u8 control_base_addr:8;
	u8 data_base_addr:8;
	u8 interrupt_source_count:3;
	u8 bits3and4:2;
	u8 function_version:2;
	u8 bit7:1;
	u8 function_number:8;
};

int do_initial_reset(struct rmi_device *rmi_dev);
int rmi_driver_f01_init(struct rmi_device *rmi_dev);

static inline void copy_pdt_entry_to_fd(struct pdt_entry *pdt,
				 struct rmi_function_descriptor *fd,
				 u16 page_start)
{
	fd->query_base_addr = pdt->query_base_addr + page_start;
	fd->command_base_addr = pdt->command_base_addr + page_start;
	fd->control_base_addr = pdt->control_base_addr + page_start;
	fd->data_base_addr = pdt->data_base_addr + page_start;
	fd->function_number = pdt->function_number;
	fd->interrupt_source_count = pdt->interrupt_source_count;
	fd->function_version = pdt->function_version;
}

#endif
