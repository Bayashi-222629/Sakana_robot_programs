/*String data;
int num=0;
int RX_PIN = 22;
int TX_PIN = 23;

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
}


void loop() {

  data = "Arduino, Can you recieve me?";
  num++;
  //Serial1.println(data);
  //Serial.println(data);
  Serial1.println(num);
  Serial.println(num);
  Serial1.flush();
  delay(2000);
}*/