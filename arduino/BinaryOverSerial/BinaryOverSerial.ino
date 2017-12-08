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

void setup() {
  Serial.begin(115200);
  /*
  Serial.print("DataPackage size: ");
  Serial.print(sizeof(DataPackage));
  Serial.println();
  Serial.print(sizeof(Data));
  Serial.println();
  randomSeed(analogRead(0));
  */
}

void loop() {
  DataPackage dp;
  dp.data.type = 0;
  dp.data.tag_address = 127;
  dp.data.anchor_address =137;
  dp.data.range = 4.5f;
  dp.data.receive_power = 1.2f;
  dp.data.first_path_power = 3.4f;
  dp.data.receive_quality = 5.6f;
  dp.data.temperature = 23.0f;
  dp.data.voltage = 5.234f;
  dp.data.carriage_return = '\r';
  dp.data.new_line = '\n';
  
  Serial.write(dp.binary,sizeof(DataPackage));

  delay(10);
}
