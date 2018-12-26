#include <Arduino.h>
#include <TimerOne.h>
//#include <L298N.h>
#include "Movement.h"

#define trigPin 12                                    // Pin 12 trigger output
#define echoPin 2                                     // Pin 2 Echo input
#define onBoardLED 13                                 // Pin 13 onboard LED
#define echo_int 0                                    // Interrupt id for echo pulse

#define TIMER_US 50                                   // 50 uS timer duration 
#define TICK_COUNTS 4000                              // 200 mS worth of timer ticks

volatile long echo_start = 0;                         // Records start of echo pulse 
volatile long echo_end = 0;                           // Records end of echo pulse
volatile long echo_duration = 0;                      // Duration - difference between end and start
volatile int trigger_time_count = 0;                  // Count down counter to trigger pulse time
volatile long range_flasher_counter = 0;              // Count down counter for flashing distance LED



//pin definition
// Right Motor
#define EN1 3
#define IN1 8
#define IN2 7

// Left Motor
#define EN2 5
#define IN3 10
#define IN4 9

int defaultSpeed;
//int randNumber;
volatile bool clearWay = false;
volatile int distanceToHinder;
volatile int thresHold = 30;

//create a motor instance
Movement tank(EN2, IN3, IN4, EN1, IN1, IN2);

//Method declaration
void trigger_pulse();
void distance_flasher();
void timerIsr();
void echo_interrupt();

void setup() {
Serial.println("in setup");  
// HC-SR04
 pinMode(trigPin, OUTPUT);                           // Trigger pin set to output
  pinMode(echoPin, INPUT);                            // Echo pin set to input
  pinMode(onBoardLED, OUTPUT);                        // Onboard LED pin set to output
  
  Timer1.initialize(TIMER_US);                        // Initialise timer 1
  Timer1.attachInterrupt( timerIsr );                 // Attach interrupt to the timer service routine 
  attachInterrupt(digitalPinToInterrupt(2), echo_interrupt, CHANGE);  // Attach interrupt to the sensor echo input
  //used for display information
   Serial.begin(9600);
   Serial.println("Starting tank");

  defaultSpeed = 100;
  // rightMotor.setSpeed(100); // an integer between 0 and 255
  // leftMotor.setSpeed(100);
  randomSeed(analogRead(0));
}

void loop() {
Serial.print("Distance: ");
Serial.println(distanceToHinder);               // Print the distance in centimeters
 if(clearWay){
   Serial.println("Forward");
   tank.Forward(defaultSpeed);
   delay(500);
 }else{
   //noInterrupts();
  Serial.println("Stop");
   tank.Stop();
   delay(500);
   if(random(9)<5)
   {
     Serial.println("Left");
     tank.PivotLeft();
     delay(500);
   }else{
     Serial.println("Right");
     tank.PivotRight();
     delay(500);
   }
   tank.Stop();
    Serial.println("Backwards");
   tank.Backward(defaultSpeed);
   delay(1000);
   tank.Stop();
   distanceToHinder = 100;

   attachInterrupt(digitalPinToInterrupt(2), echo_interrupt, CHANGE);
   delay(500);
 }
  
  
  
 // delay(100);   // every s
   // FORWARD 3 s
  
  // delay(3000);
  // tank.Stop();
  // delay(200);

  // Left 0.5 s
  // tank.PivotLeft();
  // delay(1000);
  // tank.Stop();
  // delay(200);

  // Backward 3 s
  // tank.Backward(defaultSpeed);
  // delay(3000);
  // tank.Stop();
  // delay(200);

  // Right 0.5 s
  // tank.PivotRight();
  // delay(1000);
  // tank.Stop();
  // delay(200);

}

// --------------------------
// timerIsr() 50uS second interrupt ISR()
// Called every time the hardware timer 1 times out.
// --------------------------
void timerIsr()
{
    trigger_pulse();                                 // Schedule the trigger pulses
    distance_flasher();                              // Flash the onboard LED distance indicator
}

// --------------------------
// trigger_pulse() called every 50 uS to schedule trigger pulses.
// Generates a pulse one timer tick long.
// Minimum trigger pulse width for the HC-SR04 is 10 us. This system
// delivers a 50 uS pulse.
// --------------------------
void trigger_pulse()
{
      static volatile int state = 0;                 // State machine variable

      if (!(--trigger_time_count))                   // Count to 200mS
      {                                              // Time out - Initiate trigger pulse
         trigger_time_count = TICK_COUNTS;           // Reload
         state = 1;                                  // Changing to state 1 initiates a pulse
      }
    
      switch(state)                                  // State machine handles delivery of trigger pulse
      {
        case 0:                                      // Normal state does nothing
            break;
        
        case 1:                                      // Initiate pulse
           digitalWrite(trigPin, HIGH);              // Set the trigger output high
           state = 2;                                // and set state to 2
           break;
        
        case 2:                                      // Complete the pulse
        default:      
           digitalWrite(trigPin, LOW);               // Set the trigger output low
           state = 0;                                // and return state to normal 0
           break;
     }
}

// --------------------------
// echo_interrupt() External interrupt from HC-SR04 echo signal. 
// Called every time the echo signal changes state.
//
// Note: this routine does not handle the case where the timer
//       counter overflows which will result in the occassional error.
// --------------------------
void echo_interrupt()
{
  switch (digitalRead(echoPin))                     // Test to see if the signal is high or low
  {
    case HIGH:                                      // High so must be the start of the echo pulse
      echo_end = 0;                                 // Clear the end time
      echo_start = micros();                        // Save the start time
      break;
      
    case LOW:                                       // Low so must be the end of hte echo pulse
      echo_end = micros();                          // Save the end time
      echo_duration = echo_end - echo_start;        // Calculate the pulse duration
      distanceToHinder = echo_duration / 58;
      if((distanceToHinder < thresHold) && (distanceToHinder > 0)){
        clearWay = false;
         detachInterrupt(digitalPinToInterrupt(2));
      }else{
        clearWay = true;
      }
      break;
  }
}

// --------------------------
// distance_flasher() Called from the timer 1 timerIsr service routine.
// Flashes the onboard LED at a rate inversely proportional
// to distance. The closer it gets the higher the frequency.
// --------------------------
void distance_flasher()
{
      if (--range_flasher_counter <= 0)                // Decrement and test the flash timer
      {                                                // Flash timer time out
         if (echo_duration < 25000)                    // If the echo duration is within limits
         {
           range_flasher_counter = echo_duration * 2;  // Reload the timer with the current echo duration
         }
         else
         {
           range_flasher_counter = 25000;              // If out of range use a default
         }
         
         digitalWrite( onBoardLED, digitalRead( onBoardLED ) ^ 1 );   // Toggle the onboard LED
      }
}
