String data;
float currenttime,previoustime,rpm_L;
int tick = 0; 
int PWM= 0;
void setup() {
  Serial.begin(9600);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(3, INPUT_PULLUP);
}

void loop() {
  currenttime=millis();
  if(currenttime-previoustime > 1000)
  {
  detachInterrupt(3);
  previoustime=currenttime;
  rpm_L=(tick*60)/300;
  Serial.println(rpm_L);
  analogWrite(9,PWM);
  digitalWrite(10,LOW);
  tick=0;
  while (Serial.available()>0) {
    detachInterrupt(3);
    data = Serial.readStringUntil('\n');
    PWM = data.toInt();
  }
  }
  attachInterrupt(digitalPinToInterrupt(3), count, FALLING);
}

void count()
{
  tick++;
  }
