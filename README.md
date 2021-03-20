# winglevlr

Simple single-axis autopilot based on an ESP32, MPU-9250 and an RC servo 

## Building 

If you don't have an Arduino ESP32 environment set up

```
sudo apt-get install vagrant

vagrant init hashicorp/bionic64
vagrant up
vagrant ssh
vagrant ssh -c "sudo apt-get update; sudo apt-get -y upgrade; sudo apt-get -y dist-upgrade"
vagrant halt
vagrant up

vagrant ssh -c "sudo apt-get install -y arduino git build-essential python-serial bash-completion gnuplot"
vagrant ssh

# Install homebrew package manager
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

echo 'eval "$(/home/linuxbrew/.linuxbrew/bin/brew shellenv)"' >> ${HOME}/.profile

brew  install arduino-cli
arduino-cli config init 

# Add ESP32 stuff to external data sources
sed -i 's|additional_urls: \[\]|additional_urls: \[https://dl.espressif.com/dl/package_esp32_index.json,http://arduino.esp8266.com/stable/package_esp8266com_index.json\]|' ~/.arduino15/arduino-cli.yaml 
  
arduino-cli update
arduino-cli core install esp32:esp32
```

Then get some referenced Arduino libraries 


```
arduino-cli lib install MPU9250_asukiaaa 
arduino-cli lib install "Adafruit GFX Library"
arduino-cli lib install "Adafruit ST7735 and ST7789 Library"
arduino-cli lib install OneWireNg
arduino-cli lib uninstall SD

cd ~/Arduino/libraries 
git clone https://github.com/plerup/makeEspArduino.git
git clone https://github.com/nhatuan84/esp32-micro-sdcard.git
git clone https://github.com/mikalhart/TinyGPSPlus.git

# makeEspArduino needs needs a preferences.txt file 
echo sketchbook.path=${HOME}/Arduino >> ~/.arduino15/preferences.txt
```

Finally download and build the code

```
cd ~
git clone https://github.com/cowlove/winglevlr.git
cd winglevlr/
make && make simplot
```

	
