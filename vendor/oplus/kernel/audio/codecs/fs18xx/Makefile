###
# Copyright (C) Fourier Semiconductor Inc. 2016-2020. All rights reserved.
#

ifeq ($(CONFIG_SND_SOC_FS1815),)
CONFIG_SND_SOC_FS1815 := m
endif

ifneq ($(KERNELRELEASE),)

# TOPLEVEL=$(PWD)/..
# EXTRA_CFLAGS += -I$(TOPLEVEL)

# EXTRA_CFLAGS += -DDEBUG -DFSM_DEBUG
EXTRA_CFLAGS += -Wall -Werror
MODFLAGS = -fno-pic
CFLAGS_MODULE = $(MODFLAGS)
AFLAGS_MODULE = $(MODFLAGS)

snd-soc-fs1815-objs := fsm_core.o
snd-soc-fs1815-objs += fs1815.o
snd-soc-fs1815-objs += fsm_i2c.o
obj-$(CONFIG_SND_SOC_FS1815) += snd-soc-fs1815.o

else # ndef KERNELRELEASE

PWD := $(shell pwd)

MAKEARCH := $(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE)

all:
	@echo CONFIG_SND_SOC_FS1815 = $(CONFIG_SND_SOC_FS1815)
	$(MAKEARCH) -C $(KERNEL_DIR) M=$(PWD) modules

clean:
	$(MAKEARCH) -C $(KERNEL_DIR) M=$(PWD) clean
	rm -rf .*.cmd *.o *.mod.c *.ko .tmp_versions *.order *symvers

endif
