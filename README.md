# Arduino_GPS_Oled_Display
This project enables to display some GPS information as well as temperature on an OLED display with and Arduino Pro Micro board. It is really interesting since the hardware doesn't cost much and it's fun to learn electronics and programming.

The tiny OLED display shows:
- Time (from GPS),
- Speed (km/h),
- Max speed (km/h),
- Number of satellites from GPS, 
- Temperature (Celsius),
- Distance from reference position.

Elements required to build the project (I am providing links below if you need to purchase):
- Arduino Pro Micro board (5 volt version),
- OLED display (0.96" 128x64 I2C module),
- Temperature sensor (DS18B20),
- GPS sensor (GPS6MV1 GPS),
- Resistors (2x4.7k Ohms, 1x10k Ohms).

This project is to be used within the Araduino IDE: https://www.arduino.cc/en/Main/Software

The Arduino Pro Micro board is well described here: https://learn.sparkfun.com/tutorials/pro-micro--fio-v3-hookup-guide. This is really important to declare the type of board into the Arduino IDE (https://raw.githubusercontent.com/sparkfun/Arduino_Boards/master/IDE_Board_Manager/package_sparkfun_index.json).

The following open source libraries are used (to be copied to your 'Ardunio/Libraries' folder):
- TinyGPSPlus, used to control the GPS module: https://github.com/mikalhart/TinyGPSPlus
- Adafruit_ssd1306syp, used to control the OLED display: https://github.com/Miyeah/Arduino-Dormitory-Assistant/tree/master/library/Adafruit_ssd1306syp


The Arduino board is connected to the PC which sends the program once compiled and also enables to read the Serial output (use the serial monitor in the Araduino IDE).

The GPS has 4 PINs: GND pin to ground, VCC to 5V, TX to PIN 15 of the Arduino board and RX to two resisors. The first resistor is 4.7k Ohms, the second is 10k Ohms. The other end of the first resistor goes to PIN 14 of the Arduino board. The other end of the second resistor goes to the ground. We are using resistors because the GPS logic is using 3.3 Volts (not 5 Volts).

The Temperature sensor has 3 PINs. GND (black wire) goes to ground and VCC (red wire) goes to 5V. The other wire goes to PIN 10 of the Arduino board and also to a 4.7k Ohms resistor which other end is connected to 5 Volts.

The OLED screen has 4 PINs: GND pin to ground, VCC to 5V, the SDA PIN is connected to PIN 2 of the Arduino board and the SCL PIN is connected to PIN 3 of the Arduino board.


Example of links to buy the hardware:
- Arduino Pro Micro: http://www.ebay.fr/itm/140972980117?_trksid=p2057872.m2749.l2649&ssPageName=STRK%3AMEBIDX%3AIT
- GPS: http://eud.dx.com/product/neo-6m-gy-gps6mv1-gps-apm2-5-module-with-antenna-deep-blue-844438171
- OLED: http://www.dx.com/p/0-96-128x64-i2c-interface-blue-color-oled-display-module-board-for-arduino-419232#.V9-nAPCLR9M
- Temperature sensor: http://eud.dx.com/product/ds18b20-waterproof-digital-temperature-probe-black-silver-844204290
