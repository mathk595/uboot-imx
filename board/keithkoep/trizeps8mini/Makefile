#
# Copyright 2018 NXP
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += trizeps8mini.o
obj-$(CONFIG_VIDEO_MXS) += display.o

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing_v1r1.o
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing_v1r2.o
endif
