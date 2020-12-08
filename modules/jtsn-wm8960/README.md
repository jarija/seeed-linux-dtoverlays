# Run ReSpeaker 2-Mics Pi HAT/WM8960 on Jetson-Nano/Xavier-NX

For Jetson source R32.4.2 or JetPack Image 4.4

### 1. Clone repo
```shell
	cd <your-work-directory>
        git clone https://gitlab.com/aivero/public/seeed-linux-dtoverlays.git
	cd seeed-linux-dtoverlays
        git checkout origin/add_jetson_nx_support
```

### 2.1 Build dtbo & driver for Jetson-Nano
```shell
	export CUSTOM_MOD_LIST="jtsn-wm8960"
	make all_jetsonnano
```

### 2.2 Build dtbo & driver for Xavier-NX
```shell
	export CUSTOM_MOD_LIST="jtsn-wm8960"
	make all_xaviernx
```

### 3.1 Install driver for Jetson-Nano
```shell
	sudo -E make install_jetsonnano
```

### 3.2 Install driver for Xavier-NX
```shell
	sudo -E make install_xaviernx
```

### 4.1 Install dtbo for Jetson-Nano
```shell
        # Copy one of the following dtbo to /boot/ folder
        # Note: /boot/ folder should contain only one of following dtbo
        # For "WM8960-Audio-Codec" Only support:
	sudo cp overlays/jetsonnano/jetson-seeed-2mic-wm8960.dtbo /boot
        # For "WM8960-Audio-Codec + SPI-Led" support:
	sudo cp overlays/jetsonnano/jetson-seeed-2mic-voicecard.dtbo /boot

	sudo /opt/nvidia/jetson-io/config-by-hardware.py -n "Seeed Voice Card 2MIC"
```

### 4.2 Install dtbo for Xavier-NX
```shell
        # Copy one of the following dtbo to /boot/ folder
        # Note: /boot/ folder should contain only one of following dtbo
        # For "WM8960-Audio-Codec" Only support:
	sudo cp overlays/jetsonnano/jetson-seeed-2mic-wm8960.dtbo /boot
        # For "WM8960-Audio-Codec + SPI-Led" support:
	sudo cp overlays/xaviernx/xavier-nx-seeed-2mic-wm8960-with-led.dtbo /boot

	sudo /opt/nvidia/jetson-io/config-by-hardware.py -n "Seeed Voice Card 2MIC"
```

### 5. Reboot
```shell
	sudo reboot
```

### 6.1 Restore ALSA mixer/widgets setting for Jetson-Nano
```shell
	# must wait a momemnt the time sound card busy after login
	cd ~/seeed-linux-dtoverlays
	alsactl -f extras/wm8960_asound.state-jetson-nano restore 1
```

### 6.2 Restore ALSA mixer/widgets setting for Xavier-NX
```shell
	# must wait a momemnt the time sound card busy after login
	cd ~/seeed-linux-dtoverlays
	alsactl -f extras/wm8960_asound.state-xavier-nx restore 1
```

### 7. Capture & Playback
```shell
	arecord -D hw:1,0 -f S32_LE -r 48000 -c 2 | aplay -D hw:1,0 -f S32_LE -r 48000 -c 2
```

