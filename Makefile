obj-m += gvusb2-sound.o gvusb2-video.o

gvusb2-sound-y := gvusb2-snd.o gvusb2-core.o
gvusb2-video-y := gvusb2-vid.o gvusb2-v4l2.o gvusb2-core.o gvusb2-i2c.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
