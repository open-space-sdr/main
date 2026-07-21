obj-m += fpga-dsi.o

all:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
	sudo dtc -@ -I dts -O dtb -o /boot/firmware/overlays/fpga-dsi.dtbo fpga-dsi.dts

clean:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
