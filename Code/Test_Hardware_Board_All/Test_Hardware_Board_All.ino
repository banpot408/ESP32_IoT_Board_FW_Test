#include <Wire.h>
#include <MCP342x.h>

#define RY_1       32
#define RY_2       33
#define RY_3       14
#define RY_4       23
#define RY_5       19
#define RY_6       18

#define DI_1       36
#define DI_2       39
#define DI_3       34
#define DI_4       35

#define RXD2       26
#define TXD2       27
#define SSerialTxControl 25

#define RS485Transmit    HIGH
#define RS485Receive     LOW

uint8_t address = 0x6C;
MCP342x adc = MCP342x(address);

// Configuration settings
MCP342x::Config config(MCP342x::channel1, MCP342x::oneShot,
           MCP342x::resolution18, MCP342x::gain1);

// Configuration/status read back from the ADC
MCP342x::Config status;

unsigned long Time_tick_adc = 0;
double I01 = 0;
double I02 = 0;
double I03 = 0;

bool state_led = LOW;
bool startConversion = false;

void i2c_scan()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

void setup() {

  pinMode(RY_1, OUTPUT);
  pinMode(RY_2, OUTPUT);
  pinMode(RY_3, OUTPUT);
  pinMode(RY_4, OUTPUT);
  pinMode(RY_5, OUTPUT);
  pinMode(RY_6, OUTPUT);

  pinMode(DI_1, INPUT);
  pinMode(DI_2, INPUT);
  pinMode(DI_3, INPUT);
  pinMode(DI_4, INPUT);
  pinMode(RXD2,INPUT_PULLUP);
  
  Serial.begin(115200);  

  pinMode(SSerialTxControl, OUTPUT);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2); 

  digitalWrite(SSerialTxControl, RS485Transmit);  
  Serial2.println("Hello, world?");
  delay(50);     
  digitalWrite(SSerialTxControl, RS485Receive);

  Wire.begin();
  i2c_scan();
  
  // Reset devices
  MCP342x::generalCallReset();
  delay(1); // MC342x needs 300us to settle, wait 1ms

  Wire.requestFrom(address, (uint8_t)1);
  if (!Wire.available()) {
    Serial.print("No device found at address ");
    Serial.println(address, HEX);
    while (1)
      ;
  } 
}

void loop() {
  long value = 0;
  uint8_t err;
  
  if (Serial2.available()) {
    digitalWrite(SSerialTxControl, RS485Transmit);  // Enable RS485 Transmit
    Serial2.write(Serial2.read());
    delay(10);
    digitalWrite(SSerialTxControl, RS485Receive);  // Disable RS485 Transmit 
  }

  if (millis() - Time_tick_adc >= 2000) {
    Time_tick_adc = millis();    
    state_led =! state_led;
    read_4_20mA();
    read_adc1();
    read_adc2();
  }
  test_DIO();
}
void test_DIO(){
  digitalWrite(RY_1, state_led);
  digitalWrite(RY_2, state_led);
  digitalWrite(RY_3, state_led);
  digitalWrite(RY_4, state_led);
  digitalWrite(RY_5, state_led);
  digitalWrite(RY_6, state_led);
}
/*
void test_DIO(){
  digitalWrite(RY_1,digitalRead(DI_1));
  digitalWrite(RY_2,digitalRead(DI_2));
  digitalWrite(RY_3,digitalRead(DI_1));
  digitalWrite(RY_4,digitalRead(DI_2));
  digitalWrite(RY_5,digitalRead(DI_3));
  digitalWrite(RY_6,digitalRead(DI_4));
}
*/
void read_4_20mA(){ //CH_01
  
  long value = 0;
  MCP342x::Config status;
  // Initiate a conversion; convertAndRead() will wait until it can be read
  uint8_t err = adc.convertAndRead(MCP342x::channel1, MCP342x::oneShot,
           MCP342x::resolution16, MCP342x::gain1,
           1000000, value, status);
  if (err) {
    Serial.print("Convert error: ");
    Serial.println(err);
  }
  else {
    Serial.print("read_4_20mA: ");
    Serial.println(value);
  } 
}
void read_adc2(){//CH_03
  
  long value = 0;
  MCP342x::Config status;
  // Initiate a conversion; convertAndRead() will wait until it can be read
  uint8_t err = adc.convertAndRead(MCP342x::channel3, MCP342x::oneShot,
           MCP342x::resolution16, MCP342x::gain1,
           1000000, value, status);
  if (err) {
    Serial.print("Convert error: ");
    Serial.println(err);
  }
  else {
    Serial.print("read_adc2: ");
    Serial.println(value);
  }
}
void read_adc1(){ //CH_04
  
  long value = 0;
  MCP342x::Config status;
  // Initiate a conversion; convertAndRead() will wait until it can be read
  uint8_t err = adc.convertAndRead(MCP342x::channel4, MCP342x::oneShot,
           MCP342x::resolution16, MCP342x::gain1,
           1000000, value, status);
  if (err) {
    Serial.print("Convert error: ");
    Serial.println(err);
  }
  else {
    Serial.print("read_adc1: ");
    Serial.println(value);
  }
  
}
double map_d(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
