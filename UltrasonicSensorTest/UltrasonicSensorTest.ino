/**
   HC-SR04 Proof-of-Concept
   Created by Maalav Pandya on 10/10/2021
*/


//Declaring pin numbers for sensor data lines
int trigPin = 2;
int echoPin = 3;

void setup() {

  //Declaring input and output modes
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //Initializing Serial Monitor
  Serial.begin(9600);
  Serial.println("Program Running...\n");
}


void loop() {
  while(true) {
    getDistance();
    delay(1000);
  }
}

long getDistance() {
  //Function to return distance read by ultrasonic sensor as a long data type
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
