#
# Copyright (c) 2023, STMicroelectronics - All Rights Reserved
#
# SPDX-License-Identifier: BSD-3-Clause
#

include plat/st/common/common.mk

CRASH_REPORTING		:=	1
ENABLE_PIE		:=	1

# Default Device tree
DTB_FILE_NAME		?=	stm32mp257f-ev.dtb

STM32MP25		:=	1

# STM32 image header version v2.2
STM32_HEADER_VERSION_MAJOR:=	2
STM32_HEADER_VERSION_MINOR:=	2

# Number of TF-A copies in the device
STM32_TF_A_COPIES		:=	2

# PLAT_PARTITION_MAX_ENTRIES must take care of STM32_TF-A_COPIES and other partitions
# such as metadata (2) and fsbl-m (2) to find all the FIP partitions (default is 2).
PLAT_PARTITION_MAX_ENTRIES	:=	$(shell echo $$(($(STM32_TF_A_COPIES) + 6)))

ifeq (${PSA_FWU_SUPPORT},1)
# Number of banks of updatable firmware
NR_OF_FW_BANKS			:=	2
NR_OF_IMAGES_IN_FW_BANK		:=	1

FWU_MAX_PART = $(shell echo $$(($(STM32_TF_A_COPIES) + 2 + $(NR_OF_FW_BANKS))))
ifeq ($(shell test $(FWU_MAX_PART) -gt $(PLAT_PARTITION_MAX_ENTRIES); echo $$?),0)
$(error "Required partition number is $(FWU_MAX_PART) where PLAT_PARTITION_MAX_ENTRIES is only \
$(PLAT_PARTITION_MAX_ENTRIES)")
endif
endif

# Download load address for serial boot devices
DWL_BUFFER_BASE 	?=	0x87000000

# Device tree
BL2_DTSI		:=	stm32mp25-bl2.dtsi
FDT_SOURCES		:=	$(addprefix ${BUILD_PLAT}/fdts/, $(patsubst %.dtb,%-bl2.dts,$(DTB_FILE_NAME)))

# Macros and rules to build TF binary
STM32_TF_STM32		:=	$(addprefix ${BUILD_PLAT}/tf-a-, $(patsubst %.dtb,%.stm32,$(DTB_FILE_NAME)))
STM32_LD_FILE		:=	plat/st/stm32mp2/${ARCH}/stm32mp2.ld.S
STM32_BINARY_MAPPING	:=	plat/st/stm32mp2/${ARCH}/stm32mp2.S

STM32MP_FW_CONFIG_NAME	:=	$(patsubst %.dtb,%-fw-config.dtb,$(DTB_FILE_NAME))
STM32MP_FW_CONFIG	:=	${BUILD_PLAT}/fdts/$(STM32MP_FW_CONFIG_NAME)
FDT_SOURCES		+=	$(addprefix fdts/, $(patsubst %.dtb,%.dts,$(STM32MP_FW_CONFIG_NAME)))
# Add the FW_CONFIG to FIP and specify the same to certtool
$(eval $(call TOOL_ADD_PAYLOAD,${STM32MP_FW_CONFIG},--fw-config))

# Enable flags for C files
$(eval $(call assert_booleans,\
	$(sort \
		STM32MP25 \
)))

$(eval $(call assert_numerics,\
	$(sort \
		PLAT_PARTITION_MAX_ENTRIES \
		STM32_TF_A_COPIES \
)))

$(eval $(call add_defines,\
	$(sort \
		DWL_BUFFER_BASE \
		PLAT_PARTITION_MAX_ENTRIES \
		STM32_TF_A_COPIES \
		STM32MP25 \
)))

# Include paths and source files
PLAT_INCLUDES		+=	-Iplat/st/stm32mp2/include/

PLAT_BL_COMMON_SOURCES	+=	plat/st/stm32mp2/stm32mp2_private.c

PLAT_BL_COMMON_SOURCES	+=	drivers/st/uart/${ARCH}/stm32_console.S

PLAT_BL_COMMON_SOURCES	+=	lib/cpus/${ARCH}/cortex_a35.S

PLAT_BL_COMMON_SOURCES	+=	drivers/st/bsec/bsec3.c					\
				drivers/st/reset/stm32mp2_reset.c			\
				plat/st/stm32mp2/${ARCH}/stm32mp2_helper.S		\
				plat/st/stm32mp2/stm32mp2_syscfg.c

PLAT_BL_COMMON_SOURCES	+=	drivers/st/clk/clk-stm32-core.c				\
				drivers/st/clk/stm32mp2_clk.c

BL2_SOURCES		+=	plat/st/stm32mp2/plat_bl2_mem_params_desc.c		\
				plat/st/stm32mp2/stm32mp2_fconf_firewall.c

BL2_SOURCES		+=	plat/st/stm32mp2/bl2_plat_setup.c

BL2_SOURCES		+=	drivers/st/rif/stm32mp2_risaf.c


ifneq ($(filter 1,${STM32MP_EMMC} ${STM32MP_SDMMC}),)
BL2_SOURCES		+=	drivers/st/mmc/stm32_sdmmc2.c
endif

BL2_SOURCES		+=	plat/st/stm32mp2/plat_image_load.c

# BL31 sources
BL31_SOURCES		+=	${FDT_WRAPPERS_SOURCES}

BL31_SOURCES		+=	plat/st/stm32mp2/bl31_plat_setup.c			\
				plat/st/stm32mp2/stm32mp2_pm.c				\
				plat/st/stm32mp2/stm32mp2_topology.c
# Generic GIC v2
include drivers/arm/gic/v2/gicv2.mk

BL31_SOURCES		+=	${GICV2_SOURCES}					\
				plat/common/plat_gicv2.c				\
				plat/st/common/stm32mp_gic.c

# Generic PSCI
BL31_SOURCES		+=	plat/common/plat_psci_common.c

# Compilation rules
include plat/st/common/common_rules.mk
