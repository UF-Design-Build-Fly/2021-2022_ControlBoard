//Datasheet: https://cdn.sparkfun.com/assets/8/a/f/a/c/16977-TFMini-S_-_Micro_LiDAR_Module-Product_Manual.pdf
//Example that helped figure out issues using TF mini library, although there were still a few problems with the source, resorted to just reading the data over UART.
//https://how2electronics.com/how-to-use-tfmini-s-lidar-distance-sensor-with-arduino/
//Will need to better document the code in the future. 

// wiki for seeeduino XIAO SAMD21: https://wiki.seeedstudio.com/Seeeduino-XIAO/


/*
 * 
 * Needs to be implemented: 
 *  
 *    Anything on the PCB that hasn't been accounted for or mapped out
 *    Some type of override
 *    Maybe some calibration section before the servo starts to be driven
 *    select line for manual control or user control
 *    need to figure out how many channels will be required on the receiver
 *    maybe use a switch to switch between controlling the airplane and controlling the mechanism (this will be the user control)
 *    and then a third option on the switch will turn it to automated control where it waits for a trigger from the SH switch on the transmitter
 *    ....
 *    ....
 */


#include<Servo.h>

const uint8_t L = 10;     //Number of samples in array
double dist[L];           //actual distance measurements of LiDAR (use of array was for velocity solver), just kept it as an artifact during testing and degugging, not technically needed.
int strength = 0;         //signal strength of LiDAR
int checkSum = 0;         //save check value
int uart[9];              //save data measured by LiDAR
const byte HEADER = 0x59; //frame header of data package
double dx = 0;
uint8_t arrayPosition = 0; // position within dist array

bool objectInView, runServo = false;

Servo myServo;

//Used for velocity solver (NOT USED)
int16_t dt,t1,t2 = 0;
int16_t averageDist_newVal = 0;
int16_t averageDist_preVal = 0;
double velocity = 0;

bool newData = 0; // flag is set when a set of 10 pieces of data are available to use.

#define commandPin 1 // pin PWM signal to control when the automated deployment sequence begins.
#define servoPin 2 // pin used to send control signal to (will need to implement a select line pin to control which source is driving the signal for the servo, user or microcontroller)

uint16_t commandSignal = 0;

/* TESTING NOTES
 * 
 * Best implementation would be to take advantage of the deadzone of the lidar module (TF mini S module is really an infrared module. TF stands for time of flight.)
 * and mount the lidar module pointed towards the side of the fuselage but close enough toward the vial mechanism to where when a crate drops, it will drop into the
 * FOV of the lidar module. This will give a clean and clear signal peak of when the crate entered and left the FOV of the TF-mini-s module. There should be a 
 * negative and positive peak. This sequence of events can be used to verify that a crate was actually seen to have passed in front of the module. 
 */

void setup() {
  SerialUSB.begin(115200);
  SerialUSB.print("initializing...");
  pinMode(commandPin, INPUT);
  pinMode(servoPin, OUTPUT);
  myServo.attach(servoPin);
  myServo.writeMicroseconds(1500); // set continuous servo not to move;
}
 
void loop() {
  
  commandSignal = pulseIn(commandPin, HIGH, 30000); // reads channel from receiver
  SerialUSB.println(commandSignal); //prints period of PWM to serial monitor

  if (commandSignal >= 1500) { // if SH switch has been triggered (2000us), then do the following

    Serial1.begin(115200); // initialize UART
    runServo = true; // command signal sets deployment flag (starts sequence)
    myServo.writeMicroseconds(1000); // set continuous servo to rotate    
  
    while(runServo == true) {
      get_data();

      // Super basic peak detection

      // NOTE: may also want to incorparate something that checks to see if distance is zero (something is in front of device already)
      // Need to add override to stop deployment sequence, however, this is proving difficult due to the fact we are constantly reading UART data for TF-mini-s during the
      // deployment sequence. Since TF-mini-s is constantly spitting out data at 100Hz, our data frame will be constantly out of sink if we are constantly checking some
      // PWM signal using the pulseIn() function. Will probably need to end up periodically checking the command channel on the receiver
      
      if (dx >= 3 and objectInView == true) { //dx>=3 refers to the minimum peak level in order to trigger this event
        SerialUSB.print(" Object has moved out of FOV\t");
        objectInView = false;
        SerialUSB.print("objectInView = ");
        SerialUSB.print(objectInView);
        
        runServo = false; // flag set to stop running servo/deployment
        myServo.writeMicroseconds(1500);
        Serial1.end(); // disable serial port
        
      }
      else if (dx <= -3 and objectInView == false) { //dx>=-3 refers to the minimum peak level in order to trigger this event
        SerialUSB.print(" Object has moved into of FOV\t");
        objectInView = true;
        SerialUSB.print("objectInView = ");
        SerialUSB.print(objectInView);
      }
    }
  }
}

void get_data() {
  // Datasheet says we will see two header frames with the value of 0x59 before we see any data frames.
  // Therefore, we must wait to see two frames of 0x59 in order to know which bytes we are reading.
  
  if (Serial1.available()) {                //check if serial port has data input
  
    if (Serial1.read() == HEADER) {      //assess data package frame header 0x59
 
      uart[0] = HEADER;
      while(!(Serial1.available())); // waits for a new frame to be available
      if (Serial1.read() == HEADER)      //assess data package frame header 0x59
      {
        uart[1] = HEADER;

        for (int i = 2; i < 9; i++)         //save data in array
        {
          while(!(Serial1.available()));
          uart[i] = Serial1.read();
        }
        checkSum = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
        if (uart[8] == (checkSum & 0xff))        //verify the received data as per protocol
        {

          // NOTE: NEED TO ADD DETECTION FOR WHEN DISTANCE VALUE IS NOT RELIABLE
          
          dist[arrayPosition] = uart[2] + uart[3] * 256;     //calculate distance value and increment array number
          
          strength = uart[4] + uart[5] * 256; //calculate signal strength value

          // find dx between every dist value
          if (arrayPosition == 0) {
            dx = dist[arrayPosition] - dist[L-1];
          }
          else {
            dx = dist[arrayPosition] - dist[arrayPosition-1];
          }
                 
          SerialUSB.print("dx = ");
          SerialUSB.print(dx);
          SerialUSB.print("\t");

          }

          SerialUSB.println();

          /* //print values for debugging
          SerialUSB.print("dist = ");
          SerialUSB.print(dist[arrayPosition]);                 //output measure distance value of LiDAR
          SerialUSB.print('\t');
          SerialUSB.print("averageDist_newVal = ");
          SerialUSB.print(averageDist_newVal);
          SerialUSB.print('\t'); 
          SerialUSB.print("arrayPosition = ");
          SerialUSB.print(arrayPosition);                 
          SerialUSB.print('\t');
          SerialUSB.print("dx = ");
          SerialUSB.print(dx);                
          SerialUSB.print('\t');
          SerialUSB.print("dt = ");
          SerialUSB.print(dt);     
          SerialUSB.print('\t');
          SerialUSB.print("velocity = ");
          SerialUSB.print(velocity);
          SerialUSB.print('\t');
          SerialUSB.print("strength = ");
          SerialUSB.print(strength);             //output signal strength value
          SerialUSB.print('\n');
          */

          arrayPosition++; //increment array position
        }
        
        else {
          SerialUSB.print("CHECK SUM ERROR");
          /*
          SerialUSB.println("BAD FRAME:");
          SerialUSB.print(uart[0]);
          SerialUSB.print(", ");
          SerialUSB.print(uart[1]);
          SerialUSB.print(", ");
          SerialUSB.print(uart[2]);
          SerialUSB.print(", ");
          SerialUSB.print(uart[3]);
          SerialUSB.print(", ");
          SerialUSB.print(uart[4]);
          SerialUSB.print(", ");
          SerialUSB.print(uart[5]);
          SerialUSB.print(", ");
          SerialUSB.print(uart[6]);
          SerialUSB.print(", ");
          SerialUSB.print(uart[7]);
          SerialUSB.print(", ");
          SerialUSB.println(uart[8]);
          */
        }
      }
    }
    else {
      
    }
    if (arrayPosition == L) { // if the end of the array has been reached, set arrayPosition back to zero and use data to calculate new value
      arrayPosition = 0;
      newData = true;
    }
}


/*********************************************************
 * 
 * NOTE: Not practical or accurate for our application
 * 
 * Find velocity of object moving this implementation is difficult to achieve in our application due to low speeds and accuracy of device. 
 * looking at dx alone would be a better approach as we are looking for when a crate moves out of view. This would create a spike for dx
 * 
 ********************************************************/

double velocity_solver() { 
  
  double result = 0; // cm/s
  
  if (newData == true) {
    newData = false; //clear flag
    averageDist_preVal = averageDist_newVal; //store previous values
    t1 = t2;
    
    for (int i = 0; i < L; i++) {
      averageDist_newVal += dist[i];
    }
    
    averageDist_newVal /= L; // find average
    dx = averageDist_newVal - averageDist_preVal; //find change of x
    t2 = millis();
    dt = t2 - t1;
    
    result = dx/(dt/1000.0); //  cm/s
  }
  return result;
}
