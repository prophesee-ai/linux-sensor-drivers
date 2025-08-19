obj-m := imx636.o genx320-driver.o
genx320-driver-objs += \
	genx320.o \
	psee_controls.o \
	genx320_controls.o \
	drivers/genx320/genx320_roi.o \
	drivers/genx320/genx320_roi_pixel.o \
	drivers/genx320/genx320_erc.o \
	drivers/genx320/genx320_bias.o \
	drivers/genx320/genx320_mipi.o \
	drivers/genx320/genx320.o

SRC := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
