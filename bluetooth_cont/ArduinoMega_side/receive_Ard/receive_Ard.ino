String rec_Data;

void setup() {
  Serial.begin(9600);
}
void loop() {
  if (Serial.available() > 0) {
    rec_Data = Serial.readString();
    //data = Serial.read();
    Serial.println("from ESP32");
    Serial.println(rec_Data);
  }
  delay(1000);
}