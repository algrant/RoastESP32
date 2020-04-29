/**************
 *  
 *  RoastESP32 - NodeMCU ESP32 thermocouple reader based on Arduino TC4
 *  *******************************************************************
 *  
 *  Resources
 *  =========
 *  TC-4
 *  https://github.com/FilePhil/TC4-Emulator
 *  https://github.com/greencardigan/TC4-shield/blob/master/applications/Artisan/aArtisan/trunk/src/aArtisan/commands.txt
 *  https://www.youtube.com/watch?v=0-Co-pXF2NM
 *  DHT22
 *  https://randomnerdtutorials.com/esp32-dht11-dht22-temperature-humidity-sensor-arduino-ide/
 *  MAX6675
 *  https://arduino.stackexchange.com/questions/37193/multiple-3-wire-spi-sensor-interfacing-with-arduino
 *  SPI
 *  https://www.thaieasyelec.com/article-wiki/embedded-electronics-application/09-espino32-spi.html
 *  
 *  Command sequence from artisan
 *  =============================
 *  CHAN;1200
 *  UNITS;C
 *  FILT;70;70;70;70
 *  READ
 *  https://www.home-barista.com/home-roasting/configuring-artisan-pid-t32351.html
 *  
 */

#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
uint8_t temprature_sens_read();


#include <SerialCommands.h> // https://github.com/ppedro74/Arduino-SerialCommands
#include <BluetoothSerial.h> // Classic Bluetooth
#include <DHT.h> // DHT22
#include <SPI.h> // MAX6675 over hardware SPI

// Bluetooth
#define ROAST_ESP32_BLUETOOTH_NAME "RoastESP32" //Bluetooth device name
BluetoothSerial SerialBT;

// DHT22
#define DHTPIN                 22 // GPIO22 -> DHT22 Output
#define DHTTYPE             DHT22
DHT dht(DHTPIN, DHTTYPE);
double humidity;
double ambientc;
double ambientf;

// MAX6675
#define TC1_CS                 32 // GPIO32 -> CS MAX6675[TC1]
#define TC2_CS                 33 // GPIO33 -> CS MAX6675[TC2]
#define TC3_CS                 25 // GPIO25 -> CS MAX6675[TC3]
#define TC4_CS                 26 // GPIO26 -> CS MAX6675[TC4]
/* Note: All MAX6675
 *  MAX6675 to  EPS32
 *  VCC     ->  3.3V
 *  GND     ->  GND
 *  SCK     ->  GPIO18/CLK
 *  SO      ->  GPIO19/MISO
 */

bool unit_F               = false;


// DHT22
void readDHT(){
  float h = dht.readHumidity();
  float c = dht.readTemperature();
  float f = dht.readTemperature(true);

  if(!isnan(h)){
    humidity = h;
  }
  if(!isnan(c)){
    ambientc = c;
  }
  if(!isnan(f)){
    ambientf = f;
  }
}

// MAX6675
double readCelsius(uint8_t cs) {
  uint16_t v;

  digitalWrite(cs, LOW);
  v = SPI.transfer(0x00);
  v <<= 8;
  v |= SPI.transfer(0x00);
  digitalWrite(cs, HIGH);

  if (v & 0x4) {
    // uh oh, no thermocouple attached!
    return NAN; 
  }

  v >>= 3;

  return v*0.25;
}

double readFahrenheit(uint8_t cs) {
  return readCelsius(cs) * 1.8 + 32;
}


// USB & Bluetooth SerialCommands
char serialbt_cmds_buffer[32];
char serial_cmds_buffer[32];
SerialCommands serialbt_cmds(&SerialBT, serialbt_cmds_buffer, sizeof(serialbt_cmds_buffer), "\n", ";");
SerialCommands serial_cmds(&Serial, serial_cmds_buffer, sizeof(serial_cmds_buffer), "\n", ";");

//This is the default handler, and gets called when no other command matches. 
void cmd_unrecognized(SerialCommands* sender, const char* cmd){
  sender->GetSerial()->print("Unrecognized command [");
  sender->GetSerial()->print(cmd);
  sender->GetSerial()->println("]");
}

void cmdSetChannel(SerialCommands* sender){
  sender->GetSerial()->println("#OK");
}
SerialCommand serialCmdSetChannel("CHAN", cmdSetChannel);

void cmdSetUnits(SerialCommands* sender){
  char* units = sender->Next();
  if (units[0] == 'F'){
    unit_F = true;
    sender->GetSerial()->println("#OK Farenheit");
  }else if (units[0] == 'C'){
    unit_F = false;
    sender->GetSerial()->println("#OK Celsius");
  }
}
SerialCommand serialCmdSetUnits("UNITS", cmdSetUnits);

void cmdSetFilter(SerialCommands* sender){
  sender->GetSerial()->println("#OK");
}
SerialCommand serialCmdSetFilter("FILT", cmdSetFilter);

void cmdRead(SerialCommands* sender){
  readDHT();
  if(unit_F){
    sender->GetSerial()->print(ambientf);
    sender->GetSerial()->print(",");
    sender->GetSerial()->print(readFahrenheit(TC1_CS));
    sender->GetSerial()->print(",");
    sender->GetSerial()->print(readFahrenheit(TC2_CS));
    sender->GetSerial()->print(",");
    sender->GetSerial()->print(readFahrenheit(TC3_CS));
    sender->GetSerial()->print(",");
    sender->GetSerial()->print(readFahrenheit(TC4_CS));
    sender->GetSerial()->print(",0.00,0.00,0.00,"); // Heater, Fan, PID set value
    sender->GetSerial()->print(temprature_sens_read()); // Internal temp
  }else{
    sender->GetSerial()->print(ambientc);
    sender->GetSerial()->print(",");
    sender->GetSerial()->print(readCelsius(TC1_CS));
    sender->GetSerial()->print(",");
    sender->GetSerial()->print(readCelsius(TC2_CS));
    sender->GetSerial()->print(",");
    sender->GetSerial()->print(readCelsius(TC3_CS));
    sender->GetSerial()->print(",");
    sender->GetSerial()->print(readCelsius(TC4_CS));
    sender->GetSerial()->print(",0.00,0.00,0.00,"); // Heater, Fan, PID set value
    sender->GetSerial()->print((temprature_sens_read() - 32) / 1.8); // Internal temp
  }
  sender->GetSerial()->println("");
}
SerialCommand serialCmdRead("READ", cmdRead);


void setup(){
  // DHT22 for ambient and humidity
  pinMode(DHTPIN, INPUT);
  dht.begin();

  // Thermocouple (MAX6675 x4 over hardware SPI)
  pinMode(TC1_CS, OUTPUT);
  pinMode(TC2_CS, OUTPUT);
  pinMode(TC3_CS, OUTPUT);
  pinMode(TC4_CS, OUTPUT);
  digitalWrite(TC1_CS, HIGH);
  digitalWrite(TC2_CS, HIGH);
  digitalWrite(TC3_CS, HIGH);
  digitalWrite(TC4_CS, HIGH);
  SPI.begin();
  SPI.beginTransaction (SPISettings (1000000, MSBFIRST, SPI_MODE0));

  // USB Serial
  Serial.begin(115200);
  serial_cmds.SetDefaultHandler(cmd_unrecognized);
  serial_cmds.AddCommand(&serialCmdSetChannel);
  serial_cmds.AddCommand(&serialCmdSetUnits);
  serial_cmds.AddCommand(&serialCmdSetFilter);
  serial_cmds.AddCommand(&serialCmdRead);
  // Bluetooth Serial
  SerialBT.begin(ROAST_ESP32_BLUETOOTH_NAME);
  serialbt_cmds.SetDefaultHandler(cmd_unrecognized);
  serialbt_cmds.AddCommand(&serialCmdSetChannel);
  serialbt_cmds.AddCommand(&serialCmdSetUnits);
  serialbt_cmds.AddCommand(&serialCmdSetFilter);
  serialbt_cmds.AddCommand(&serialCmdRead);
}

void loop(){
  serialbt_cmds.ReadSerial();
  serial_cmds.ReadSerial();
}
