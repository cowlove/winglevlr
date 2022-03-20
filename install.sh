#!/bin/bash
# on host machine:
# sudo apt-get install vagrant apt-cacher-ng
# vagrant init hashicorp/bionic64
# vagrant up
# vagrant ssh -c "curl -fsSL https://raw.githubusercontent.com/cowlove/winglevlr/master/install.sh | sh -x"

# TODO: check if proxy is up/exists
if [ -d /vagrant ]; then
	echo 'Acquire::http { Proxy "http://10.0.2.2:3142"; };' | sudo tee /etc/apt/apt.conf.d/01proxy
fi

sudo apt-get update
#sudo apt-get -y upgrade; sudo apt-get -y dist-upgrade
sudo apt-get install -y arduino git curl build-essential bash-completion gnuplot python3 python3-pip
sudo apt-get install python-serial

mkdir -p ${HOME}/bin
export BINDIR=${HOME}/bin
export PATH=$PATH:${HOME}/bin

arduino-cli version || curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

arduino-cli config init 
sed -i 's|additional_urls: \[\]|additional_urls: \[https://dl.espressif.com/dl/package_esp32_index.json,http://arduino.esp8266.com/stable/package_esp8266com_index.json\]|' ~/.arduino15/arduino-cli.yaml 
arduino-cli update
arduino-cli core install esp32:esp32
arduino-cli lib install MPU9250_asukiaaa 
arduino-cli lib install "Adafruit GFX Library"
arduino-cli lib install "Adafruit ST7735 and ST7789 Library"
arduino-cli lib install OneWireNg
arduino-cli lib uninstall SD
mkdir -p ${HOME}/Arduino/libraries
cd ${HOME}/Arduino/libraries 
git clone git@github.com:cowlove/esp32jimlib.git
git clone https://github.com/plerup/makeEspArduino.git
cd makeEspArduino
git checkout 190e073
cd ..
git clone https://github.com/nhatuan84/esp32-micro-sdcard.git
git clone https://github.com/mikalhart/TinyGPSPlus.git
git clone https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library.git

sudo pip3 install -U pyserial gmplot matplotlib


# makeEspArduino needs needs a preferences.txt file 
echo sketchbook.path=${HOME}/Arduino >> ~/.arduino15/preferences.txt
   
 
