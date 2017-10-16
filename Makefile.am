ACLOCAL_AMFLAGS=-I m4

SUBDIRS = inc

#The sdi-device_drivers library
lib_LTLIBRARIES = libopx_sdi_device_drivers.la

libopx_sdi_device_drivers_la_SOURCES = \
	src/sdi_cpld.c \
	src/sdi_cpld_pin.c \
	src/sdi_cpld_pin_group.c \
	src/sdi_dell_eeprom.c \
	src/sdi_eeprom.c \
	src/sdi_emc142x.c \
	src/sdi_emc2305.c \
	src/sdi_i2cmux_pin.c \
	src/sdi_io_bus.c \
	src/sdi_max6620.c \
	src/sdi_max6699.c \
	src/sdi_media_phy_mgmt.c \
	src/sdi_mono_color_pin_led.c \
	src/sdi_nvram.c \
	src/sdi_onie_eeprom.c \
	src/sdi_pmbus_dev.c \
	src/sdi_pseudo_bus.c \
	src/sdi_qsfp.c \
	src/sdi_qsfp_4X1_1000baseT.c \
	src/sdi_qsfp_eeprom.c \
	src/sdi_s6k_psu.c \
	src/sdi_seven_segment_pin_led.c \
	src/sdi_sf_entity_info.c \
	src/sdi_sf_fan.c \
	src/sdi_sf_io_bus.c \
	src/sdi_sf_tmp.c \
	src/sdi_sfp.c \
	src/sdi_sfp_eeprom.c \
	src/sdi_tmp75.c \
	src/sys-interface-drivers/sdi_gpio.c \
	src/sys-interface-drivers/sdi_i2cdev.c \
	src/sys-interface-drivers/sdi_sysfs_gpio_helpers.c \
	src/sdi_i2cmux_pca.c \
	src/sdi_segment_display.c


libopx_sdi_device_drivers_la_LDFLAGS = -lopx_sdi_framework -lopx_common -lopx_logging -lz -lm -lpthread -lrt -version-info 1:1:0
libopx_sdi_device_drivers_la_CFLAGS = -I$(top_srcdir)/inc/opx -I$(top_srcdir)/inc/opx/eeprom -I$(includedir)/opx


