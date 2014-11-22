obj-$(CONFIG_RMI4_BUS) += rmi_bus.o
obj-$(CONFIG_RMI4_I2C) += rmi_i2c.o
obj-$(CONFIG_RMI4_SPI) += rmi_spi.o
# don't change the order of rmi_f34.o and rmi_driver.o
# the seamless firmware flash depends on this sequence
obj-$(CONFIG_RMI4_GENERIC) += rmi_f34.o rmi_driver.o rmi_f01.o
obj-$(CONFIG_RMI4_F09) += rmi_f09.o
obj-$(CONFIG_RMI4_F1A) += rmi_f1a.o
obj-$(CONFIG_RMI4_F11) += rmi_f11.o
obj-$(CONFIG_RMI4_F17) += rmi_f17.o
obj-$(CONFIG_RMI4_F19) += rmi_f19.o
obj-$(CONFIG_RMI4_F54) += rmi_f54.o
obj-$(CONFIG_RMI4_DEV) += rmi_dev.o

ifeq ($(KERNELRELEASE),)

# KERNELDIR ?= /home/<AndroidKernelDirectory>
PWD := $(shell pwd)

.PHONY: build clean

build:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c

endif
