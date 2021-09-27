#!/bin/bash
vagrant ssh apt-get update
vagrant ssh apt-get install -y git
vagrant ssh git clone https://github.com/cowlove/winglevlr.git
vagrant ssh ~/winglevlr/setupVag.sh


sudo apt-get update; sudo apt-get -y upgrade; sudo apt-get -y dist-upgrade
sudo apt-get install -y arduino git build-essential python-serial bash-completion gnuplot

curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

export PATH=$PATH:~/bin

arduino-cli config init 
sed -i 's|additional_urls: \[\]|additional_urls: \[https://dl.espressif.com/dl/package_esp32_index.json,http://arduino.esp8266.com/stable/package_esp8266com_index.json\]|' ~/.arduino15/arduino-cli.yaml 
arduino-cli update
arduino-cli core install esp32:esp32
arduino-cli lib install MPU9250_asukiaaa 
arduino-cli lib install "Adafruit GFX Library"
arduino-cli lib install "Adafruit ST7735 and ST7789 Library"
arduino-cli lib install OneWireNg
arduino-cli lib uninstall SD
cd ~/Arduino/libraries 
git clone https://github.com/plerup/makeEspArduino.git
git clone https://github.com/nhatuan84/esp32-micro-sdcard.git
git clone https://github.com/mikalhart/TinyGPSPlus.git
git clone https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library.git
   
cd ~
git clone https://github.com/cowlove/winglevlr.git
cd winglevlr/
make && make simplot
 
