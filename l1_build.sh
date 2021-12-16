#!/bin/bash
source /home/pi/VerifTest/LinuxDriverTests/trunk/vl53l1_testes/unexportGpio.sh
export DRIVER_DIR=/home/pi/VerifTest/LinuxDriver/trunk/driver/vl53L1/

cd ${DRIVER_DIR}
make clean
make
sudo dtoverlay -R
sudo dtoverlay stmvl53l1

cd /home/pi/VerifTest/LinuxDriver/trunk/android/hardware/vl53l1_test
make clean
make

cd /home/pi/VerifTest/LinuxDriverTests/trunk/vl53l1_test
make clean
make
cd /home/pi/VerifTest/LinuxDriverTests/trunk/vl53l1_testes
source setDirforTest.sh
