for port in ls -1 /dev/tty*
do
	if [[ $port == *"USB"* ]]; then
		python3 edit_config.py
		cp ../bin/data/config_default.json ../bin/data/config_live.json
		./mklittlefs -s 1441792 -c ../bin/data ../bin/littlefs.bin

                python3 esptool.py --port $port --chip esp32 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect 0x1000 ../bin/MainSrc.ino.bootloader.bin 0x8000 ../bin/MainSrc.ino.partitions.bin 0xe000 ../bin/boot_app0.bin 0x10000 ../bin/MainSrc.ino.bin 

		python3 esptool.py --port $port --chip esp32 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect 0x00290000 ../bin/littlefs.bin 
	fi
	
done  
