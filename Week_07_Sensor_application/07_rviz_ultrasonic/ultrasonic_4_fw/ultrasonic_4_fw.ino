// sensor 1
int trig1 = 2;
int echo1 = 3;

// sensor 2
int trig2 = 4;
int echo2 = 5;

// sensor 3
int trig3 = 6;
int echo3 = 7;

// sensor 4
int trig4 = 8;
int echo4 = 9;

void setup() {
  Serial.begin(9600);

  pinMode(trig1, OUTPUT); 
  pinMode(echo1, INPUT);
  
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);
  
  pinMode(trig3, OUTPUT);
  pinMode(echo3, INPUT);
  
  pinMode(trig4, OUTPUT);
  pinMode(echo4, INPUT);
}
 
void loop() {
  long duration1, distance1;
  long duration2, distance2;
  long duration3, distance3;
  long duration4, distance4;
  
  digitalWrite(trig1, LOW);
  delayMicroseconds(2);
  digitalWrite(trig1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig1, LOW);

  duration1 = pulseIn(echo1, HIGH);
  distance1 = (340 * (duration1 / 2.0)) / 1000;
  
  digitalWrite(trig2, LOW);
  delayMicroseconds(2);
  digitalWrite(trig2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig2, LOW);

  duration2 = pulseIn(echo2, HIGH);
  distance2 = (340 * (duration2 / 2.0)) / 1000;

  digitalWrite(trig3, LOW);
  delayMicroseconds(2);
  digitalWrite(trig3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig3, LOW);

  duration3 = pulseIn(echo3, HIGH);
  distance3 = (340 * (duration3 / 2.0)) / 1000;

  digitalWrite(trig4, LOW);
  delayMicroseconds(2);
  digitalWrite(trig4, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig4, LOW);

  duration4 = pulseIn(echo4, HIGH);
  distance4 = (340 * (duration4 / 2.0)) / 1000;

  Serial.print(distance1);
  Serial.print("mm ");
  Serial.print(distance2);
  Serial.print("mm ");
  Serial.print(distance3);
  Serial.print("mm ");
  Serial.print(distance4);
  Serial.println("mm");

  delay(100);
}
