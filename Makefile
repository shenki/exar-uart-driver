obj-m := vizzini.o

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD       := $(shell pwd)

EXTRA_CFLAGS	:= -DDEBUG=0

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)

modules_install:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions vtty

install-rules: 99-vizzini.rules
	cp $< "/etc/udev/rules.d/99-vizzini.rules"
	udevadm control --reload-rules
	udevadm trigger --attr-match=idVendor=04e2 --verbose
