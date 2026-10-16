ifneq ($(KERNELRELEASE),)
# kbuild part

obj-m += gvusb2-sound.o gvusb2-video.o

gvusb2-sound-y := gvusb2-snd.o gvusb2-core.o
gvusb2-video-y := gvusb2-vid.o gvusb2-v4l2.o gvusb2-core.o gvusb2-i2c.o

else
# normal part

KDIR ?= /lib/modules/$(shell uname -r)/build

# must match PACKAGE_VERSION in dkms.conf
GVUSB2_VERSION := $(shell sed -n 's/^PACKAGE_VERSION="\(.*\)"/\1/p' dkms.conf)
DKMS_SRC := /usr/src/gvusb2-$(GVUSB2_VERSION)

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean

# Install through dkms so the modules are rebuilt on kernel updates.
install:
	# remove other versions, which would install modules with the same names
	for v in $$(dkms status -m gvusb2 | sed -n 's|^gvusb2/\([^,:]*\).*|\1|p' | sort -u); do \
		[ "$$v" = "$(GVUSB2_VERSION)" ] || { dkms remove gvusb2/$$v --all; rm -rf /usr/src/gvusb2-$$v; }; \
	done
	install -d $(DKMS_SRC)
	install -m 644 Makefile dkms.conf *.c *.h $(DKMS_SRC)/
	dkms status -m gvusb2 -v $(GVUSB2_VERSION) | grep -q . || \
		dkms add gvusb2/$(GVUSB2_VERSION)
	# install --force alone reuses a cached build, so rebuild explicitly
	dkms build gvusb2/$(GVUSB2_VERSION) --force
	dkms install gvusb2/$(GVUSB2_VERSION) --force
	@echo "Installed srcversion: $$(modinfo -F srcversion gvusb2_video)"
	@echo "Local build srcversion: $$(modinfo -F srcversion ./gvusb2-video.ko 2>/dev/null || echo '(not built)')"
	@echo "If the old modules are loaded, replugging won't replace them. Reload with:"
	@echo "  sudo modprobe -r gvusb2_video gvusb2_sound && sudo modprobe -a gvusb2_video gvusb2_sound"

uninstall:
	-dkms remove gvusb2/$(GVUSB2_VERSION) --all
	rm -rf $(DKMS_SRC)

.PHONY: all clean install uninstall

endif
