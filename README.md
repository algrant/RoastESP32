# RoastESP32

Took advantage from TC4 protocol with a little twist... BLUETOOTH

![Wiring Diagram](https://raw.githubusercontent.com/hokoon/RoastESP32/master/images/wiring.png)

## Requirement

### Hardware
1. ESP32 Dev Kit DOIT (ESP-WROOM-32 30 Pins)
2. MAX6675 Module x4
3. DHT22 Module
4. 10K-Ohm Resistor

### Software
1. [Arduino IDE](https://www.arduino.cc/en/main/software)

### Library
1. [SerialCommands](https://github.com/ppedro74/Arduino-SerialCommands)
2. [DHT](https://github.com/adafruit/DHT-sensor-library)

## Installation

### Arduino IDE
1. Download and install on your local machine
2. Click "Sketch > Include Library > Manage Libraries"
3. Search and install "SerialCommands" and "DHT"

### Build
1. Plug the board to usb port
2. Select Board click "Tools > Board > DOIT ESP32 DEVKIT V1"
3. Select Port click "Tools > Port > /dev/cu.SLAB_USBtoUART" (this is macOS, if you use other OS please look for similar device)
4. Copy content from RoastESP32.ino to your project or clone this project to your workspace
5. Click "Upload" button on the top left of the IDE
6. When the IDE show "Connecting..." push-hold the "BOOT" button on the board, then push-release "EN", then release "BOOT"


## Usage

### Artisan
1. Click "Config > Device" and choose "TC4" and uncheck "PID Firmware"

### USB
- Artisan Click "Config > Port", Comm Port "/dev/cu.SLAB_USBtoUART", Baud Rate "115200"

### Bluetooth
- Plug the ESP32 to usb charger
- Open Bluetooth Preferences...
- Connect to "RoastESP32"
- Artisan Click "Config > Port", Comm Port "/dev/cu.RoastESP32-ESP32SPP", Baud Rate "115200"

Enjoy!
> - Anuwat Hokoon

![Assembled 1](https://raw.githubusercontent.com/hokoon/RoastESP32/master/images/assembled1.jpg =1080x)
![Assembled 2](https://raw.githubusercontent.com/hokoon/RoastESP32/master/images/assembled2.jpg =1080x)
![Screen Shot](https://raw.githubusercontent.com/hokoon/RoastESP32/master/images/screenshot.png =1080x)


