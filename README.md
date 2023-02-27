BME280-Environmental-Sensor from seengreat:www.seengreat.com
 =======================================
# Ⅰ  Instruction
## 1.1、Product Overview
This product is a high-precision Environmental sensor, the BME280 sensor to achieve temperature, humidity and barometric pressure monitoring, the chip temperature sensing in -40~+85 °C, humidity sensing in 0~100%, barometric pressure sensing in 300~1100hPa. PH2.0 6PIN wire is used to connect the module to the development board for environmental detection, Arduino and Raspberry Pi C and python and STM32 versions of demo codes are available, demo codes enable real-time monitoring of temperature, humidity and barometric pressure.<br>
## 1.2、Product parameters
|Size	|30mm(Length)*18mm(Width)|
|-----------|-------------------------------|
|sensor chip	|BME280|
|Signal interface	|I2C/SPI|
|Supply voltage	|3.3V/5V|
|Temperature sensing	|-40~85°C (resolution 0.01°C, accuracy ±1°C)|
|Humidity sensing	|0~100%RH (resolution 0.008%RH, accuracy ±3%RH)|
|Barometric Pressure sensing	|300~1100hPa (resolution 0.18Pa, accuracy ±1hPa)|	
## 1.3、Product dimensions
30mm(Length)*18mm(Width)<br>
# Ⅱ  Usage
## 2.1、Hardware interface configuration instructions
### 2.1.1、BME280 sensor wiring instructions
|PIN	|I2C	|SPI|
|-----------|----------|----|
|VCC	|Power supply positive (3.3V/5V)	|Power supply positive (3.3V/5V)|
|GND	|Power supply ground	|Power supply ground|
|SCK	|clock line	|clock input|
|MOSI	|data line	|data input|
|MISO/ADDR	|NC	|data output|
|CS	|NC	|SPI chip selection pin (active at low level)|
## 2.2、Raspberry Pi Demo Codes Usage	
### 2.2.1、Wiring instructions
The demo codes in the Raspberry Pi motherboard uses the wiringpi number pin definition and the definition of the wiring with the Raspberry Pi motherboard is shown in the table below.<br>
|BME280 	|I2C	|SPI	|BCM Number	|wiringPi Number|
|-----------|-----------|-----------|----------------------|-------------------|
|VCC	|3.3V/5V	|3.3V/5V	|3.3V/5V	|3.3V/5V|
|GND	|GND	|GND	|GND	|GND|
|SCK	|SCL	|SCK	|11	|14|
|MOSI	|SDA	|MOSI	|10	|12|
|MISO/ADDR	|NC	|MISO	|9	|13|
|CS	|NC	|GPIO.6	|25	|6|

Table2-2 Pin definition of the module wiring to the Raspberry Pi<br>
The program defaults to I2C wiring mode and changes #define USE_IIC 1 in the main.c program from 1 to 0 when SPI is required.<br>
### 2.2.2、Wiringpi library installation
   sudo apt-get install wiringpi<br>
   wget https://project-downloads.drogon.net/wiringpi-latest.deb  # Version 4B upgrade of Raspberry Pi<br>
   sudo dpkg -i wiringpi-latest.deb<br>
   gpio -v # If version 2.52 appears, the installation is successful<br>
#For the Bullseye branch system, use the following command:<br>
git clone https://github.com/WiringPi/WiringPi<br>
cd WiringPi<br>
./build<br>
gpio -v<br>
#Running gpio - v will result in version 2.70. If it does not appear, it indicates an installation error<br>
If the error prompt "ImportError: No module named 'wiringpi'" appears when running the python version of the sample program, run the following command.<br>
#For Python 2. x version<br>
pip install wiringpi<br>
#For Python version 3. X<br>
pip3 install wiringpi<br>
Note: If the installation fails, you can try the following compilation and installation:<br>
git clone --recursive https://github.com/WiringPi/WiringPi-Python.git<br>
Note: The -- recursive option can automatically pull the submodule, otherwise you need to download it manually.<br>
Enter the WiringPi Python folder you just downloaded, enter the following command, compile and install:<br>
#For Python 2. x version<br>
sudo python setup.py install <br>
#For Python version 3. X<br>
sudo python3 setup.py install<br>
If  error occurs:<br>

At this time, enter the command sudo apt install swig to install swig. After that, compile and install sudo python3 setup.py install. If a message similar to the following appears, the installation is successful.<br>
### 2.2.3、Open SPI interface
sudo raspi-config<br>
Enable I2C interface:<br>
Interfacing Options > I2C> Yes<br>
Enable SPI interface:<br>
Interfacing Options > SPI > Yes<br>
Restart the device.<br>
sudo reboot<br>
Run the command to check whether I2C and SPI are started.<br>
lsmod<br>
If i2c_bcm2835 and spi_bcm2835 are displayed, it means I2C, the SPI module is started.<br>
Install the i2c-tools tool to confirm<br>	
sudo apt-get install i2c-tools<br>
View connected I2C devices<br>
i2cdetect -y 1<br>
The connection between BME280 and Raspberry Pi is successful. ADDR is connected to high level by default, and the address is 0X77. If ADDR is connected to low power level, the address is 0X76.<br>
Open the directory demo codes\raspberry-pi, copy the c folder to the Raspberry Pi, open the Raspberry Pi terminal into the C folder, and then run the following command.<br>
sudo make clean<br>
sudo make<br>
sudo ./main<br>
Real-time temperature, humidity, and barometric pressure information can be seen at the Raspberry Pi terminal.<br>
If SPI driven: Wire the BME280 module according to the SPI bus wiring method in the interface description, change the USEIIC macro definition in the main.c file to 0, and then recompile and run the program.<br>
## 2.3 Arduino Demo Codes Usage
### 2.3.1、Wiring instructions
The wiring between the BME280 and the Arduino development board is shown in the table below:<br>
|BME280 	|I2C	|SPI|
|-----------|-----------|----|
|VCC	|3.3V/5V	|3.3V/5V|
|GND	|GND	|GND|
|SCK	|SCL	|D13|
|MOSI	|SDA	|D11|
|MISO/ADDR	|NC	|D12|
|CS	|NC	|D10|

Table2-3 Pin definition of the module wiring to the Arduino<br>
The program defaults to I2C wiring mode, change #define USE_IIC 1 from 1 to 0 when SPI is required, and then recompile and run the program.<br>
### 2.3.2、Installation of libraries
Open the directory demo codes\Arduino\bme280, open the bme280.ino file with Arduino software, and install the Adafruit Unified Sensor library and Adafruit's BME280 library. To install libraries, navigate to Sketch > Include Library > Manage Libraries and wait for the library manager to download the library index and update the list of installed libraries.<br>

__Thank you for choosing the products of Shengui Technology Co.,Ltd. For more details about this product, please visit:
www.seengreat.com__

