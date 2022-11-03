void setup() {
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  currentTime = micros();
  loopTimer   = currentTime; 　

}

void loop() {
  while (1) {
    currentTime = micros();
    if (currentTime - loopTimer >= 4000)break; //250Hz(4000us)
  }
  loopTimer = currentTime;
  pinMode(9, HIGH);
  pinMode(10, HIGH);
  PWMLoopTimer = micros();

  PWM1_timer = 1000 + PWMLoopTimer;
  PWM2_timer = 1250 + PWMLoopTimer;

  while (digitalRead(9) + digitalRead(10)> 0) {
    PWMLoopTimer = micros();
    if (PWM1_timer <= PWMLoopTimer)digitalWrite(9, LOW);
    if (PWM2_timer <= PWMLoopTimer)digitalWrite(10, LOW);
  }

}
