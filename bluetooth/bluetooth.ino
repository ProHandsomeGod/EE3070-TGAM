
void setup() {
  hw827Setup();
  tgamSetup();
  //Serial.begin(115200);
  //print(30.5, 2);
}

void loop() {
  //delay(1000);
  hw827Loop();
  tgamLoop();
}
