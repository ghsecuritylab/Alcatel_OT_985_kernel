obj-$(CONFIG_VIDEO_GO7007) += go7007.o
obj-$(CONFIG_VIDEO_GO7007_USB) += go7007-usb.o
obj-$(CONFIG_VIDEO_GO7007_USB_S2250_BOARD) += s2250.o s2250-loader.o
obj-$(CONFIG_VIDEO_GO7007_SAA7113) += wis-saa7113.o
obj-$(CONFIG_VIDEO_GO7007_OV7640) += wis-ov7640.o
obj-$(CONFIG_VIDEO_GO7007_SAA7115) += wis-saa7115.o
obj-$(CONFIG_VIDEO_GO7007_TW9903) += wis-tw9903.o
obj-$(CONFIG_VIDEO_GO7007_UDA1342) += wis-uda1342.o
obj-$(CONFIG_VIDEO_GO7007_SONY_TUNER) += wis-sony-tuner.o
obj-$(CONFIG_VIDEO_GO7007_TW2804) += wis-tw2804.o

go7007-objs += go7007-v4l2.o go7007-driver.o go7007-i2c.o go7007-fw.o \
		snd-go7007.o

s2250-objs += s2250-board.o

# Uncomment when the saa7134 patches get into upstream
#ifneq ($(CONFIG_VIDEO_SAA7134),)
#obj-$(CONFIG_VIDEO_SAA7134) += saa7134-go7007.o
#EXTRA_CFLAGS += -Idrivers/media/video/saa7134 -DSAA7134_MPEG_GO7007=3
#endif

# S2250 needs cypress ezusb loader from dvb-usb
ifneq ($(CONFIG_VIDEO_GO7007_USB_S2250_BOARD),)
EXTRA_CFLAGS += -Idrivers/media/dvb/dvb-usb
endif

EXTRA_CFLAGS += -Idrivers/media/dvb/frontends
EXTRA_CFLAGS += -Idrivers/media/dvb/dvb-core
