#define DATA_FORMAT_JSON
//#define DATA_FORMAT_BINARY

#include <SPI.h>
#include <DW1000Ranging.h>
#ifdef DATA_FORMAT_JSON
#include <ArduinoJson.h>
#endif

enum class Events : byte { NEW_RANGE = 0, NEW_DEVICE = 1, INACTIVE_DEVICE = 2, };

const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 3; // irq pin
const uint8_t PIN_SS = SS; // spi select pin
const char ADDRESS[] = "A0:00:00:00:B1:6B:00:B5";

void setup() {
  Serial.begin(115200);
  // while(!Serial) {
  //   delay(10);
  // }
  // delay(1000);
  
  //init the configuration
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  //Enable the filter to smooth the distance
  //DW1000Ranging.useRangeFilter(true);
  
  //we start the module as a tag
  DW1000Ranging.startAsTag(ADDRESS, DW1000.MODE_LONGDATA_RANGE_ACCURACY, false);
}

void loop() {
  DW1000Ranging.loop();
}

void newRange() {
  transmitDATA(Events::NEW_RANGE, DW1000Ranging.getDistantDevice());
}

void newDevice(DW1000Device* device) {
  transmitDATA(Events::NEW_DEVICE, device);
}

void inactiveDevice(DW1000Device* device) {
  transmitDATA(Events::INACTIVE_DEVICE, device);
}

#ifdef DATA_FORMAT_JSON
void transmitDATA(const Events event, const DW1000Device* device) {
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  const byte* shortAddress = DW1000Ranging.getCurrentShortAddress();

  root["type"] = static_cast<byte>(event);
  root["tagAdr"] = shortAddress[1]*256+shortAddress[0];
  root["ancAdr"] = device->getShortAddress();
  root["range"] = (event==Events::NEW_RANGE) ? device->getRange() : 0.0f;
  root["rxPower"] = (event==Events::NEW_RANGE) ? device->getRXPower() : 0.0f;
  root["fpPower"] = (event==Events::NEW_RANGE) ? device->getFPPower() : 0.0f;
  root["quality"] = (event==Events::NEW_RANGE) ? device->getQuality() : 0.0f;
  root.printTo(Serial);
  Serial.println();
}
#endif

#ifdef DATA_FORMAT_BINARY
#pragma pack(push, 1)
struct Data
{
  unsigned char type;
  short tag_address;
  short anchor_address;
  float range;
  float receive_power;
  float first_path_power;
  float receive_quality;
  float temperature;
  float voltage;
  unsigned char carriage_return;
  unsigned char new_line;
};

union DataPackage
{
  Data data;
  unsigned char binary[sizeof( Data )];
};
#pragma pack(pop)

void transmitDATA(const Events event, const DW1000Device* device) {
  if(event!=Events::NEW_RANGE)
    return;

  const byte* shortAddress = DW1000Ranging.getCurrentShortAddress();
  float temp, vbat;
  DW1000.getTempAndVbat(temp, vbat);
  
  DataPackage dp;
  dp.data.type = static_cast<unsigned char>(event);
  dp.data.tag_address = shortAddress[1]*256+shortAddress[0];
  dp.data.anchor_address = device->getShortAddress();;
  dp.data.range = device->getRange();
  dp.data.receive_power = device->getRXPower();
  dp.data.first_path_power = device->getFPPower();
  dp.data.receive_quality = device->getQuality();
  dp.data.temperature = temp;
  dp.data.voltage = vbat;
  dp.data.carriage_return = '\r';
  dp.data.new_line = '\n';

  Serial.write(dp.binary, sizeof(DataPackage));
  
  //Serial.print(dp.data.type); Serial.print(",");
  //Serial.print(dp.data.tag_address); Serial.print(",");
  //Serial.print(dp.data.anchor_address); Serial.print(",");
  //Serial.print(dp.data.range); Serial.print(",");
  //Serial.print(dp.data.receive_power); Serial.print(",");
  //Serial.print(dp.data.first_path_power); Serial.print(",");
  //Serial.print(dp.data.receive_quality); Serial.print(",");
  //Serial.print(dp.data.temperature); Serial.print(",");
  //Serial.print(dp.data.voltage); Serial.println();
}
#endif