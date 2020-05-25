#!/bin/sh
../tools/mkimage -A arm -T script -O linux -d  boot.debian.pConXS-7inch.txt             boot.debian.pConXS-7inch.scr
../tools/mkimage -A arm -T script -O linux -d  boot.debian.pConXS-7inch-fat.txt         boot.debian.pConXS-7inch-fat.scr
../tools/mkimage -A arm -T script -O linux -d  boot.debian.pConXS-7inch-fat-debug.txt   boot.debian.pConXS-7inch-fat-debug.scr
../tools/mkimage -A arm -T script -O linux -d  boot.debian.pConXS-7inch-ext4.txt        boot.debian.pConXS-7inch-ext4.scr
../tools/mkimage -A arm -T script -O linux -d  boot.debian-installer.txt                boot.debian-installer.scr
../tools/mkimage -A arm -T script -O linux -d  boot.debian-installer-ext4.txt           boot.debian-installer-ext4.scr
