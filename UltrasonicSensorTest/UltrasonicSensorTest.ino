/**
   HC-SR04 Proof-of-Concept
   Created by Maalav Pandya on 10/10/2021
*/

int trigPin = 2;
int echoPin = 3;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.begin(9600);
  Serial.println("Program Running...\n");
}


void loop() {
  while(true) {
    Serial.println(getDistance());
    delay(1000);
  }
}

long getDistance() {
  long distance, duration;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH); //Reads length of HIGH signal recieved by echoPin in microseconds
  distance = ((duration * 0.034) / 2) / 2.54; //Dividing by final 2.54 to convert from cm -> in
  Serial.println(distance);
  return distance;
}
