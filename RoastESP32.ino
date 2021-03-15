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

#include <SerialCommands.h> // https://github.com/ppedro74/Arduino-SerialCommands
#include <BluetoothSerial.h> // Classic Bluetooth
#include <DHT.h> // DHT22
#include <SPI.h> // MAX6675 over hardware SPI

// Bluetooth
#define ROAST_ESP32_BLUETOOTH_NAME "RoastESP32_CAM" //Bluetooth device name
BluetoothSerial SerialBT;

// DHT22
#define DHTPIN                 22 // GPIO22 -> DHT22 Output
#define DHTTYPE             DHT22
DHT dht(DHTPIN, DHTTYPE);
double humidity;
double ambientc;
double ambientf;

// MAX6675
#define TC1_CS                 12 // GPIO12 -> CS MAX6675[TC1]
#define TC2_CS                 13 // GPIO13 -> CS MAX6675[TC2]
#define TC3_CS                 15 // GPIO15 -> CS MAX6675[TC3]
#define TC4_CS                 26 // GPIO26 -> CS MAX6675[TC4] <- I don't have a fourth max6675 so whatever...
/* Note: All MAX6675
 *  MAX6675 to  EPS32
 *  VCC     ->  3.3V
 *  GND     ->  GND
 *  SCK     ->  GPIO14/CLK  <-- for ESP32-CAM
 *  SO      ->  GPIO2/MISO  <-- for ESP32-CAM
 */

#define ESP32_CAM_CLK 14
#define ESP32_CAM_MISO 2


bool unit_F               = false;

#define SMA                     5
int sma_idx                   = 0;
bool sma_filled           = false;
double tc1s[SMA], tc2s[SMA], tc3s[SMA], tc4s[SMA];
double tc1, tc2, tc3, tc4;

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

bool readTCs(){
  tc1s[sma_idx] = readCelsius(TC1_CS);
  tc2s[sma_idx] = readCelsius(TC2_CS);
  tc3s[sma_idx] = readCelsius(TC3_CS);
  tc4s[sma_idx] = readCelsius(TC4_CS);
  if(!isnan(tc1s[sma_idx]) && !isnan(tc1s[sma_idx]) && !isnan(tc1s[sma_idx]) && !isnan(tc1s[sma_idx])){
    sma_idx++;
    if(sma_idx >= SMA){
      sma_filled = true;
      sma_idx = 0;
    }
    tc1 = 0;
    tc2 = 0;
    tc3 = 0;
    tc4 = 0;
    if(sma_filled){
      for(int i = 0; i<SMA; i++){
        tc1 += tc1s[i];
        tc2 += tc2s[i];
        tc3 += tc3s[i];
        tc4 += tc4s[i];
      }
      tc1 /= SMA;
      tc2 /= SMA;
      tc3 /= SMA;
      tc4 /= SMA;
    }
    return true;
  }
  return false;
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
    sender->GetSerial()->print(tc1 * 1.8 + 32); // double down on tc1 cause I have no ambient
    sender->GetSerial()->print(",");
    sender->GetSerial()->print(tc1 * 1.8 + 32);
    sender->GetSerial()->print(",");
    sender->GetSerial()->print(tc2 * 1.8 + 32);
    sender->GetSerial()->print(",");
    sender->GetSerial()->print(tc3 * 1.8 + 32);
    sender->GetSerial()->print(",");
    sender->GetSerial()->print(0 * 1.8 + 32); // don't send nans for missing tc4
    sender->GetSerial()->print(",0.00,0.00,0.00"); // Heater, Fan, PID set value
  }else{
    sender->GetSerial()->print(tc1); // double down on tc1 cause I have no ambient
    sender->GetSerial()->print(",");
    sender->GetSerial()->print(tc1);
    sender->GetSerial()->print(",");
    sender->GetSerial()->print(tc2);
    sender->GetSerial()->print(",");
    sender->GetSerial()->print(tc3);
    sender->GetSerial()->print(",");
    sender->GetSerial()->print(0); // don't send nans for missing tc4
    sender->GetSerial()->print(",0.00,0.00,0.00"); // Heater, Fan, PID set value
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
  SPI.begin(ESP32_CAM_CLK, ESP32_CAM_MISO);
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

  sma_idx = 0;
  sma_filled = true;
  tc1 = 0;
  tc2 = 0;
  tc3 = 0;
  tc4 = 0;
}

void loop(){
  serialbt_cmds.ReadSerial();
  serial_cmds.ReadSerial();
  if(readTCs()){
    delay(200);
  }
}
