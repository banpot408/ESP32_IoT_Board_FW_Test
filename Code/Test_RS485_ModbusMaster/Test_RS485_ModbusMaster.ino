#include <ModbusMaster.h>

#define RXD2       26
#define TXD2       27
#define SSerialTxControl 25

#define RS485Transmit    HIGH
#define RS485Receive     LOW

#define MAX485_DE      25

ModbusMaster node;


void preTransmission()
{
  digitalWrite(SSerialTxControl, RS485Transmit);

}

void postTransmission()
{
  digitalWrite(SSerialTxControl, RS485Receive);

}


void setup() {
  
  Serial.begin(115200);  

  pinMode(SSerialTxControl, OUTPUT);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2); 

  digitalWrite(SSerialTxControl, RS485Receive);  

  node.begin(2, Serial2);
  delay(1); // MC342x needs 300us to settle, wait 1ms
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}

void loop()
{
  uint8_t result;
  uint16_t data[2]; // prepare variable of storage data from sensor
   
  Serial.println("get data");
  result = node.readInputRegisters(1, 2); // Read 2 registers starting at 1)
  if (result == node.ku8MBSuccess)
  {
    Serial.print("Temp: ");
    Serial.println(node.getResponseBuffer(0)/10.0f);
    Serial.print("Humi: ");
    Serial.println(node.getResponseBuffer(1)/10.0f);
  }
  delay(500);
}
