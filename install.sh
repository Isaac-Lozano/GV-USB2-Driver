# Unload drivers if they are already loaded
grep -q "gvusb2_video" /proc/modules && sudo rmmod -f gvusb2_video
grep -q "gvusb2_sound" /proc/modules && sudo rmmod -f gvusb2_sound
make clean && make && \
    sudo mkdir -p /lib/modules/$(uname -r)/kernel/drivers/media/usb/gvusb2/ && \
    zstd *.ko && \
    sudo mv -vt /lib/modules/$(uname -r)/kernel/drivers/media/usb/gvusb2/ *.ko.zst && \
    sudo depmod $(uname -r) && \
    sudo modprobe -v gvusb2-video && \
    sudo modprobe -v gvusb2-sound
