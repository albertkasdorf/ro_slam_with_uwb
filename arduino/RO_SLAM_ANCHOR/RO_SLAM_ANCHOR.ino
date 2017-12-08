#include <SPI.h>
#include <DW1000Ranging.h>

const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 3; // irq pin
const uint8_t PIN_SS = SS; // spi select pin
const char ADDRESS[] = "B1:00:00:00:B1:6B:00:B5";

void setup() {
  Serial.begin(115200);
  //while(!Serial) {
  //  delay(10);
  //}
  //delay(1000);
  
  //init the configuration
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);
  //DW1000Ranging.attachNewRange(newRange);
  //DW1000Ranging.attachBlinkDevice(newBlink);
  //DW1000Ranging.attachInactiveDevice(inactiveDevice);
  //Enable the filter to smooth the distance
  //DW1000Ranging.useRangeFilter(true);

  // General configuration
  DW1000.setAntennaDelay(16435);

  // LED configuration
  DW1000.enableDebounceClock();
  DW1000.enableLedBlinking();
  DW1000.setGPIOMode(MSGP0, LED_MODE); // enable GPIO0/RXOKLED blinking
  DW1000.setGPIOMode(MSGP1, LED_MODE); // enable GPIO1/SFDLED blinking
  DW1000.setGPIOMode(MSGP2, LED_MODE); // enable GPIO2/RXLED blinking
  DW1000.setGPIOMode(MSGP3, LED_MODE); // enable GPIO3/TXLED blinking
  
  //we start the module as an anchor
  DW1000Ranging.startAsAnchor(ADDRESS, DW1000.MODE_LONGDATA_RANGE_ACCURACY, false);
}

void loop() {
  DW1000Ranging.loop();
}

void newRange() {
  Serial.print("from: "); Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  Serial.print("\t Range: "); Serial.print(DW1000Ranging.getDistantDevice()->getRange()); Serial.print(" m");
  Serial.print("\t RX power: "); Serial.print(DW1000Ranging.getDistantDevice()->getRXPower()); Serial.println(" dBm");
}

void newBlink(DW1000Device* device) {
  Serial.print("blink; 1 device added ! -> ");
  Serial.print(" short:");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device* device) {
  Serial.print("delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}

