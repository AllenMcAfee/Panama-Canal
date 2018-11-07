// PanamaControl3.pde
// -*- mode: C++ -*-
//
// 2/17/15 - Start of the Panama Canal Control Program
// ***** Note motors must be RELEASED when movement is done!! They heat up otherwise!!!
// Initially, will just do "automatic" when button is pressed.
//  Boat will move to first lock, the water will rise up with the boat, and reset.
//  Reset will just move boat back, no home switch.
//====================================================================================
// Assuming the first part works, now add the DC motor control for the gates to Lock 1.
//  - boat must move to lock 1 entrance, entry gate is at closed position (on reset)
//  - entry gate goes to lowered position
//  - boat moves into lock 1
//  - entry gate goes to fill position
//  - boat and water move up together
//====================================================================================
// Assuming the second part works, this part adds the 3 buttons and leds.
//  - GREEN is the light that says OK to proceed. Press this to move to the entrance of the next lock.
//  - RED is the light that says "Wait". Pressing this button causes the boat to move into the next lock.
//  - WHITE is the light for AUTOMATIC mode in action. To enter automatic mode, press the button.
//    - AUTOMATIC mode is entered after X seconds of idleness (always, or just after reset??)
//    - If a time out is needed, then it should be tied to a timer interrupt?
//  - Press GREEN and WHITE together to cause a RESET.
//=====================================================================================
// Assuming the third part works, this part sets up arrays to do all locks
//  - may need separate handling for the locks that lower the boat? (DONE)
//---------------------------------------------------------------------------------------
// 2/27/15  Need to fix pointer handling and the arrays.  OK
//


#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include "panamaTypes.h"

// Comment this out to suppress debug messages.
#define DEBUG 1

// These control looping and reset behavior. Change these only for debugging.
#define STEPPER_COUNT 8
#define GATE_COUNT  8
//#define STEPPER_COUNT 8
//#define GATE_COUNT  8

//-------------------------------------------------------------------------------------------------
// First stepper on first board is the boat horizontal movement
// Second stepper on first board is the boat vertical movement
// Third stepper is water in Lock 1, Etc.
// Need to instantiate each of 4 controllers for steppers
// These MUST match the physical reality!!!
Adafruit_MotorShield AFMS_0 = Adafruit_MotorShield(); 
Adafruit_MotorShield AFMS_1 = Adafruit_MotorShield(0x61); 
Adafruit_MotorShield AFMS_2 = Adafruit_MotorShield(0x62); 
Adafruit_MotorShield AFMS_3 = Adafruit_MotorShield(0x63);
// For the DC motors
// Need to instantiate each of 2 controllers for DC motors
// These MUST match the physical reality!!!
Adafruit_MotorShield AFMS_4 = Adafruit_MotorShield(0x64);
Adafruit_MotorShield AFMS_5 = Adafruit_MotorShield(0x66);
// Connect the steppers with 200 steps per revolution (1.8 degree)
// to the shields. This must reflect physical reality of stepper connections.
// NOTE: It is probably better to remap a motor to the correct board location than
//  to change the placement of motor numbers in the arrays!!!!
Adafruit_StepperMotor *myStepper1 = AFMS_0.getStepper(200, 1);
Adafruit_StepperMotor *myStepper2 = AFMS_0.getStepper(200, 2);
Adafruit_StepperMotor *myStepper3 = AFMS_1.getStepper(200, 1);
Adafruit_StepperMotor *myStepper4 = AFMS_1.getStepper(200, 2);
Adafruit_StepperMotor *myStepper5 = AFMS_2.getStepper(200, 1);
Adafruit_StepperMotor *myStepper6 = AFMS_2.getStepper(200, 2);
Adafruit_StepperMotor *myStepper7 = AFMS_3.getStepper(200, 1);
Adafruit_StepperMotor *myStepper8 = AFMS_3.getStepper(200, 2);
// wrappers for the steppers forward/backward one step functions
// NOTE: If motor directins need to be reversed, it can be done here.
//  It's probably better to swap wires on the controllers, but that
//  may be more difficult.
void forwardstep1() {  
  myStepper1->onestep(FORWARD, DOUBLE);  }
void backwardstep1() {  
  myStepper1->onestep(BACKWARD, DOUBLE);  }
void forwardstep2() {  
  myStepper2->onestep(FORWARD, DOUBLE);  }
void backwardstep2() {  
  myStepper2->onestep(BACKWARD, DOUBLE);  }
void forwardstep3() {  
  myStepper3->onestep(FORWARD, DOUBLE);  }
void backwardstep3() {  
  myStepper3->onestep(BACKWARD, DOUBLE);  }
void forwardstep4() {  
  myStepper4->onestep(FORWARD, DOUBLE);  }
void backwardstep4() {  
  myStepper4->onestep(BACKWARD, DOUBLE);  }
void forwardstep5() {  
  myStepper5->onestep(FORWARD, DOUBLE);  }
void backwardstep5() {  
  myStepper5->onestep(BACKWARD, DOUBLE);  }
void forwardstep6() {  
  myStepper6->onestep(FORWARD, DOUBLE);  }
void backwardstep6() {  
  myStepper6->onestep(BACKWARD, DOUBLE);  }
void forwardstep7() {  
  myStepper7->onestep(FORWARD, DOUBLE);  }
void backwardstep7() {  
  myStepper7->onestep(BACKWARD, DOUBLE);  }
void forwardstep8() {  
  myStepper8->onestep(FORWARD, DOUBLE);  }
void backwardstep8() {  
  myStepper8->onestep(BACKWARD, DOUBLE);  }
// Wrappers for the steppers in an AccelStepper object
// DON'T change these!
AccelStepper stepper1(forwardstep1, backwardstep1);
AccelStepper stepper2(forwardstep2, backwardstep2);
AccelStepper stepper3(forwardstep3, backwardstep3);
AccelStepper stepper4(forwardstep4, backwardstep4);
AccelStepper stepper5(forwardstep5, backwardstep5);
AccelStepper stepper6(forwardstep6, backwardstep6);
AccelStepper stepper7(forwardstep7, backwardstep7);
AccelStepper stepper8(forwardstep8, backwardstep8);
//------------------------------------------------------------------------------------------------
// Need to instantiate 8 DC Motors (for the Lock Gates)
// This must reflect physical reality of motor connections.
Adafruit_DCMotor *motor1 = AFMS_4.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS_4.getMotor(2);
Adafruit_DCMotor *motor3 = AFMS_4.getMotor(3);
Adafruit_DCMotor *motor4 = AFMS_4.getMotor(4);
Adafruit_DCMotor *motor5 = AFMS_5.getMotor(1);
Adafruit_DCMotor *motor6 = AFMS_5.getMotor(2);
Adafruit_DCMotor *motor7 = AFMS_5.getMotor(3);
Adafruit_DCMotor *motor8 = AFMS_5.getMotor(4);
// -----------------------------------------------------------------------------------------------
// Arrays for moving through the locks
// DON'T change the ordering of the motors! Fix the mapping of the physical connection to name in
//  the instantiations above.
// This array is used to release the motors (remove power from them) so thay don't overheat. The AccelStepper library can't do this
//  for the scheme we use, so the Adafruit_StepperMotor methods must be used directly.
Adafruit_StepperMotor * stepperToRelease[] = {myStepper1, myStepper2, myStepper3, myStepper4, myStepper5, myStepper6, myStepper7, myStepper8};
//  The DC motors are controlled with the Adafruit library methods.
Adafruit_DCMotor *gateMotor[] = {motor1, motor2, motor3, motor4, motor5, motor6, motor7, motor8}; 
// Array for looping through the gates.
AccelStepper *waterMotor[] = {&stepper3, &stepper4, &stepper5, &stepper6, &stepper7, &stepper8};
// Array for looping during RESET.
AccelStepper *allSteppers[] = {&stepper1, &stepper2, &stepper3, &stepper4, &stepper5, &stepper6, &stepper7, &stepper8};
// Initial positions (boat and water)
// resetPosition is where the stepper moves to prior to vinding the HOME switch.
long resetPosition[] = {BOAT_X_HOME, BOAT_Y_HOME, LOCK1_HOME, LOCK2_HOME, LOCK3_HOME, LOCK4_HOME, LOCK5_HOME, LOCK6_HOME};
// Once the HOME switch is hit, the boat moves to the startPosition to begin the sequence of movement through the locks.
long startPosition[] = {BOAT_X_HOME, BOAT_Y_HOME, LOCK1_MT, LOCK2_MT, LOCK3_MT, LOCK4_MT, LOCK5_MT, LOCK6_MT};
// Boat X positions
// Wait is at the entry to the lock
long lockWaitPosition[] = {LOCK1_WAIT_POSITION, LOCK2_WAIT_POSITION, LOCK3_WAIT_POSITION, LAGOON_WAIT_POSITION,
                           LOCK4_WAIT_POSITION, LOCK5_WAIT_POSITION, LOCK6_WAIT_POSITION, OCEAN_WAIT_POSITION};
// Inside is just inside the gate where the boat stays while the lock fills or empties
long insidePosition[] = {LOCK1_INSIDE_POSITION, LOCK2_INSIDE_POSITION, LOCK3_INSIDE_POSITION, LAGOON_INSIDE_POSITION,
                          LOCK4_INSIDE_POSITION, LOCK5_INSIDE_POSITION, LOCK6_INSIDE_POSITION, OCEAN_INSIDE_POSITION};
// Water Positons - also used to control the Y-axis position of the boat.
long lockFullPosition[] = {LOCK1_FULL, LOCK2_FULL, LOCK3_FULL, LOCK4_FULL, LOCK5_FULL, LOCK6_FULL};
long lockMTPosition[] = {LOCK1_MT, LOCK2_MT, LOCK3_MT, LOCK4_MT, LOCK5_MT, LOCK6_MT};
// The reference positions are the values of the boat Y-axis which correspond to 0 position of water stepper in each lock
long lockRefPosition[] = {LOCK1_REF, LOCK2_REF, LOCK3_REF, LOCK4_REF, LOCK5_REF, LOCK6_REF};
// Gate position sensor pin numbers
byte gatePositionSensor[] = {LOCK1_ENTER_GATE_POSITION_SENSOR, LOCK2_ENTER_GATE_POSITION_SENSOR,
                              LOCK3_ENTER_GATE_POSITION_SENSOR, LAGOON_ENTER_GATE_POSITION_SENSOR,
                              LOCK4_ENTER_GATE_POSITION_SENSOR, LOCK5_ENTER_GATE_POSITION_SENSOR,
                              LOCK6_ENTER_GATE_POSITION_SENSOR, OCEAN_ENTER_GATE_POSITION_SENSOR};
// Gate positon readings
int gateOpenVal[] = {GATE1_OPEN, GATE2_OPEN, GATE3_OPEN, GATE4_OPEN,
                      GATE5_OPEN, GATE6_OPEN, GATE7_OPEN, GATE8_OPEN};
int gateRaisedVal[] = {GATE1_RAISED, GATE2_RAISED, GATE3_RAISED, GATE4_RAISED,
                        GATE5_RAISED, GATE6_RAISED, GATE7_RAISED, GATE8_RAISED};
int gateReadyVal[] = {GATE1_READY, GATE2_READY, GATE3_READY, GATE4_READY,
                      GATE5_READY, GATE6_READY, GATE7_READY, GATE8_READY};
// Home position switches
byte homeSensor[] = { BOAT_HOME_SENSE_X, BOAT_HOME_SENSE_Y, LOCK1_HOME_SENSE, LOCK2_HOME_SENSE,
                      LOCK3_HOME_SENSE, LOCK4_HOME_SENSE, LOCK5_HOME_SENSE, LOCK6_HOME_SENSE };
// Global constants
byte ledPin = 13;    // probably not needed - used for debug.
volatile int mode;
int state; 
int count = 0;      // counts number of locks
int rcount = 0;      // counts number of items reset
// This is for AUTOMATIC mode
long previousMillis = 0;      // for non-blocking time delay
long delayTime = 1000;
unsigned long currentMillis;
Adafruit_DCMotor *currGateMotor; 
//Adafruit_StepperMotor * currReleaseStepper;  // shouldn't need - use index
AccelStepper *stepper;  // used in RESET
AccelStepper *waterStepper;  // used in LOCK1 seguence
  
void setup()
{  
#if defined DEBUG
  Serial.begin(38400);
  Serial.println("Control test!");
#endif
// configure output LEDs for buttons
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin,LOW);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(GREEN_LED,HIGH);
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED,HIGH);
  pinMode(WHITE_LED, OUTPUT);
  digitalWrite(WHITE_LED,HIGH);
#if defined DEBUG
        Serial.println("init of leds done ");
#endif
// configure inputs for the buttons
   //pinMode(sw1,INPUT);
   //digitalWrite(sw1,HIGH);  // turn on pull-up resistor
   pinMode(GREEN_BUTTON,INPUT);
   digitalWrite(GREEN_BUTTON,HIGH);  // turn on pull-up resistor
   pinMode(RED_BUTTON,INPUT);
   digitalWrite(RED_BUTTON,HIGH);
   pinMode(WHITE_BUTTON,INPUT);
   digitalWrite(WHITE_BUTTON,HIGH);
  attachInterrupt(0, automaticButtonHandler, FALLING);
#if defined DEBUG
        Serial.println("init buttons done ");
#endif
   // Home position sensors initialization
 for (int i= 0; i< STEPPER_COUNT; i++){
   pinMode(homeSensor[i],INPUT);
   digitalWrite(homeSensor[i],HIGH);  // turn on pull-up resistor
 }
#if defined DEBUG
        Serial.println("init of homes done ");
#endif
//-----------------------------------------------------------
// Here we start the motor and stepper controllers. 
//  Default speed is used for the steppers.
//  A slower PWM speed is used for the motors. Allows them to run slower.
   AFMS_0.begin();
#if defined DEBUG
        Serial.println("init motor board 0 ");
#endif
   AFMS_1.begin();
#if defined DEBUG
        Serial.println("init motor board 1 ");
#endif
   AFMS_2.begin();
#if defined DEBUG
        Serial.println("init motor board 2 ");
#endif
   AFMS_3.begin();
#if defined DEBUG
        Serial.println("init motor board 3 ");
#endif
   AFMS_4.begin(MOTOR_PWM_SPEED);
#if defined DEBUG
        Serial.println("init motor board 4 ");
#endif
   AFMS_5.begin(MOTOR_PWM_SPEED);
#if defined DEBUG
        Serial.println("init motor board 5 ");
#endif
//------------------------------------------------------------ 
   state = MOVE_TO_LOCK_WAIT;
   mode = NORMAL;    // do this here to clear any interrupt glitch at start
   //mode = RESET;
#if defined DEBUG
        Serial.println("Ready to start loop ");
#endif
}

void loop()
{  
  int val;    // holds pot value for gates position
  int target;    // used for gate positioning

#if defined DEBUG
  long pos1;
  unsigned long mval;
#endif
  currentMillis = millis();    // updated each pass
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  if (mode == AUTOMATIC){
    digitalWrite(WHITE_LED, LOW);
  }
  //////////////////////////////////////////////
  /// Need to protect this better!!!!!!!!!!!!
  //// Don't go to SETUP_RESET if already doing RESET
  //// Maybe need a RESETING mode??
  if (mode == RESET){
    mode = RESETING;
    state = SETUP_RESET;
    rcount = 0;
  }
  if (count > RESET_COUNT){
    count = 0;        // must reset count
    state = SETUP_RESET;
    mode = RESETING;
    rcount = 0;
  }
// Assume boat is at start_x,start_y, water is at start_water_lock1
// On green button press:
// Boat must move to the lock entrance and turn off green led, turn on red led
// On red button press:
// Lock gate lowers, boat enters
//    gate rises to high position, and water and boat rise 
// Red led off, green on and wait for green button press.
// Repeat for three RISING locks
// On green button, boat moves across lagoon and to LOWERING lock entrance
// // Lowering sequence: On Red button press.
        // Exit gate rises
        // Water fills the lock
        // entry Gate lowers
        // boat moves into lock
        // entry gate rises
        // water and lock lower
// Repeat for three LOWERING locks
// On green button, boat moves across ocean and does RESET sequence.
// On automatic, boad moves thru each lock, pausing at each stage Lights swquence.
// On RESET , boat, gates, and water return to their start positions

      switch (state){
      case BOAT_IDLE:    // this is just a dummy state so I can force a pause in the action
        break;
      case MOVE_TO_LOCK_WAIT:
        // green led on
        // press green button to leave
        // do this process 5 times in AUTOMATIC mode  (( LATER  ))
          digitalWrite(GREEN_LED, LOW);
          digitalWrite(RED_LED, HIGH);
          if (mode == AUTOMATIC){
            if(currentMillis - previousMillis > delayTime) {
              previousMillis = currentMillis; 
#if defined DEBUG
        Serial.println("in MOVE_TO_LOCK_WAIT auto mode ");
#endif  
              state = MOVE_TO_LOCK_SETUP;
            }
          }
          if (digitalRead(GREEN_BUTTON) == LOW){
            delay (20);    // debounce
            if (digitalRead(GREEN_BUTTON) == LOW){
              state = MOVE_TO_LOCK_SETUP;
#if defined DEBUG
        Serial.println("in MOVE_TO_LOCK_WAIT got green ");
#endif
            }
          }
          break;
        case MOVE_TO_LOCK_SETUP:
          digitalWrite(ledPin, HIGH);   // set the LED on
          stepper1.setMaxSpeed(BOAT_SPEED);
          stepper1.setAcceleration(BOAT_ACCEL);
          //stepper1.moveTo(LOCK1_WAIT_POSITION);
          stepper1.moveTo(lockWaitPosition[count]);
          state = MOVE_TO_LOCK;
#if defined DEBUG
        Serial.println("in MOVE_TO_LOCK_SETUP ");
        Serial.print("count: "); Serial.println(count);
#endif
          break;
        case MOVE_TO_LOCK:
          stepper1.run();
          if (stepper1.distanceToGo() == 0) {
            myStepper1->release(); 
            state = ENTER_LOCK_WAIT;
            digitalWrite(ledPin, LOW);    // set the LED off
          }
          break;
      case ENTER_LOCK_WAIT:
        // red led on
        // press red button to enter lock
          digitalWrite(RED_LED, LOW);
          digitalWrite(GREEN_LED, HIGH);
          if (mode == AUTOMATIC){
            if(currentMillis - previousMillis > delayTime) {
              previousMillis = currentMillis; 
              if (count <= LAGOON_COUNT) {  // Raising locks are ready
                state = ENTER_LOCK_SETUP;
#if defined DEBUG
        Serial.println("in ENTER_LOCK_WAIT ");
#endif
              } else if (count == OCEAN_COUNT) {
                state = ENTER_LOCK_SETUP;    // allows direct exit to Ocean
              }else {                  // must ready the lowering locks
                state = PREP_LOWERING_LOCK_SETUP;
              }
            }
          }
          if (digitalRead(RED_BUTTON) == LOW){
            delay (20);    // debounce
            if (digitalRead(RED_BUTTON) == LOW){
              if (count <= LAGOON_COUNT) {  // Raising locks are ready
                state = ENTER_LOCK_SETUP;
#if defined DEBUG
        Serial.println("in ENTER_LOCK_WAIT blue button press ");
#endif
              } else if (count == OCEAN_COUNT) {
                state = ENTER_LOCK_SETUP;    // allows direct exit to Ocean
              } else {                  // must ready the lowering locks
                state = PREP_LOWERING_LOCK_SETUP;
              }
            }
          }
        break;
        case ENTER_LOCK_SETUP:
        // Have to lower the gate first
          digitalWrite(ledPin, HIGH);   // set the LED on
           target = gateOpenVal[count];
           val = analogRead(gatePositionSensor[count]);
           if (val > target){  // Original
           //if (val < target){
             moveDown(gateMotor[count], gatePositionSensor[count], target);
           } else {
             moveUp(gateMotor[count], gatePositionSensor[count], target);
           }
          stepper1.setMaxSpeed(BOAT_SPEED);
          stepper1.setAcceleration(BOAT_ACCEL);
          stepper1.moveTo(insidePosition[count]);
#if defined DEBUG
        Serial.println("in ENTER_LOCK_SETUP ");
        Serial.print("count: "); Serial.println(count);
#endif
          state = ENTER_LOCK;
          break;
        case ENTER_LOCK:
          stepper1.run();
          if (stepper1.distanceToGo() == 0) {
            myStepper1->release();      // Need to do this to keep motors from heating up
            // Choose raising or lowering action
            if ((count <= LAGOON_COUNT) || (count == OCEAN_COUNT)){  // Raising locks are ready
              state = RAISE_LOCK_SETUP;
            } else {                  // must ready the lowering locks
              state = LOWER_LOCK_SETUP;
            }
            digitalWrite(ledPin, LOW);    // set the LED off
          }
          break;
        case RAISE_LOCK_SETUP:
        // Have to raise the entry gate when boat is inside next section 
        // Note currGateMotor is defined in ENTER_LOCK1_SETUP
          digitalWrite(ledPin, HIGH);   // set the LED on
           target = gateRaisedVal[count];
           val = analogRead(gatePositionSensor[count]);
           if (val > target){
             moveDown(gateMotor[count], gatePositionSensor[count], target);
           } else {
             moveUp(gateMotor[count], gatePositionSensor[count], target);
           } 
          // if boat is entering the lagoon or ocean, we're done
          if ((count == LAGOON_COUNT)|| (count == OCEAN_COUNT)){
            state = MOVE_TO_LOCK_WAIT;
            count++;
            break;
          }
          waterStepper = waterMotor[count];
          stepper2.setMaxSpeed(WATER_SPEED);
          stepper2.setAcceleration(WATER_ACCEL);
          waterStepper->setMaxSpeed(WATER_SPEED);
          waterStepper->setAcceleration(WATER_ACCEL);
          stepper2.moveTo(lockRefPosition[count]+lockFullPosition[count]); 
          waterStepper->moveTo(lockFullPosition[count]);
          state = RAISE_LOCK;
#if defined DEBUG
        Serial.println("in RAISE_LOCK_SETUP ");
        Serial.print("count: "); Serial.println(count);
#endif
          break;
        case RAISE_LOCK:
          // waterStepper is defined in previous step
          stepper2.run();
          waterStepper->run();
          if ((stepper2.distanceToGo() == 0) && (waterStepper->distanceToGo() == 0)) {
#if defined DEBUG
        Serial.println("Release stepper 2 ");
#endif            
            myStepper2->release();      // Need to do this to keep motors from heating up
#if defined DEBUG
        Serial.println("Release water stepper ");
#endif
            stepperToRelease[count + 2]->release();    // Index here is to the waterStepper.
#if defined DEBUG
        Serial.println("All released ");
#endif
            state = MOVE_TO_LOCK_WAIT;
            //state = BOAT_IDLE;      // not sure what to do here, just wait
            digitalWrite(ledPin, LOW);    // set the LED off
            count++;
          }
          break;
        case PREP_LOWERING_LOCK_SETUP:
        // Raise Exit Gate
          digitalWrite(ledPin, HIGH);   // set the LED on
#if defined DEBUG
        Serial.println("in LOWERING_LOCK_SETUP ");
        Serial.print("count: "); Serial.println(count);
#endif
           target = gateRaisedVal[count];
           //target = gateRaisedVal[count+1];  Original
           /*
           val = analogRead(gatePositionSensor[count+1]);
           if (val > target){
             moveDown(gateMotor[count], gatePositionSensor[count+1], target);
           } else {
             moveUp(gateMotor[count], gatePositionSensor[count+1], target);
           }*/
           val = analogRead(gatePositionSensor[count]);
           if (val > target){
             moveDown(gateMotor[count], gatePositionSensor[count], target);
           } else {
             moveUp(gateMotor[count], gatePositionSensor[count], target);
           }
          // Fill lock with water
          waterStepper = waterMotor[count-1];
          waterStepper->setMaxSpeed(WATER_SPEED);
          waterStepper->setAcceleration(WATER_ACCEL);
          waterStepper->moveTo(lockFullPosition[count]);
          state = PREP_LOWERING_LOCK;
          break;
        case PREP_LOWERING_LOCK:
          // waterStepper is defined in previous step
          waterStepper->run();
          if (waterStepper->distanceToGo() == 0) {
            stepperToRelease[count + 1]->release();    // Index here is to the waterStepper.
            state = ENTER_LOCK_SETUP;
            //state = BOAT_IDLE;      // not sure what to do here, just wait
            digitalWrite(ledPin, LOW);    // set the LED off
          }
          break;
        case LOWER_LOCK_SETUP:
        // Have to raise the entry gate when boat is inside next section (lock, lagoon or sea)
        // entry gate rises
        // water and lock lower
          digitalWrite(ledPin, HIGH);   // set the LED on
          // 
#if defined DEBUG
        Serial.println("in LOWER_LOCK_SETUP ");
        Serial.print("count: "); Serial.println(count);
#endif
           target = gateRaisedVal[count];
           val = analogRead(gatePositionSensor[count]);
           if (val > target){
             moveDown(gateMotor[count], gatePositionSensor[count], target);
           } else {
             moveUp(gateMotor[count], gatePositionSensor[count], target);
           }
          waterStepper = waterMotor[count-1];    // may need to be count -1?? or put in a dummy waterMotor??
          stepper2.setMaxSpeed(WATER_SPEED);
          stepper2.setAcceleration(WATER_ACCEL);
          waterStepper->setMaxSpeed(WATER_SPEED);
          waterStepper->setAcceleration(WATER_ACCEL);
          stepper2.moveTo(lockRefPosition[count-1]-lockFullPosition[count-1]);
          waterStepper->moveTo(lockMTPosition[count-1]);
          state = LOWER_LOCK;
          break;
        case LOWER_LOCK:
          // waterStepper is defined in previous step
          stepper2.run();
          waterStepper->run();
          if ((stepper2.distanceToGo() == 0) && (waterStepper->distanceToGo() == 0)) {
            myStepper2->release();      // Need to do this to keep motors from heating up
            stepperToRelease[count + 1]->release();    // Index here is to the waterStepper.
            state = MOVE_TO_LOCK_WAIT;
            //state = BOAT_IDLE;      // not sure what to do here, just wait
            count++;
            digitalWrite(ledPin, LOW);    // set the LED off
            //pState = 0;    // wait for another press
          }
          break;
          
        // then need a final sequence to exit last gate ?????
        
        case SETUP_RESET:
          digitalWrite(ledPin, HIGH);   // set the LED on
          digitalWrite(RED_LED, LOW);
          digitalWrite(GREEN_LED, LOW);
          digitalWrite(WHITE_LED, LOW);
#if defined DEBUG
        Serial.println("in SETUP_RESET ");
        Serial.print("rcount: "); Serial.println(rcount);
#endif
          stepper = allSteppers[rcount];
          if (rcount == 0){      // Slower for the boat X position
            stepper->setMaxSpeed(BOAT_RESET_SPEED);
          }else{
            stepper->setMaxSpeed(RESET_SPEED);
          }
          stepper->setAcceleration(RESET_ACCEL);
          stepper->moveTo(resetPosition[rcount]);
          state = RESET_1;
          break;
        case RESET_1:
          stepper->run();
          if (stepper->distanceToGo() == 0) {
            stepperToRelease[rcount]->release();      // Need to do this to keep motors from heating up
#if defined DEBUG
        Serial.println("in RESET_1 released ");
        Serial.print("rcount: "); Serial.println(rcount);
#endif
            if (rcount++ < STEPPER_COUNT){
              state = SETUP_RESET;
            }
            else {

              state = SETUP_HOME;
              //state = SETUP_RESET2;
              rcount = 0;
       //// get rid of next six lines!!
/*          digitalWrite(RED_LED, HIGH);
          digitalWrite(GREEN_LED, HIGH);
          digitalWrite(WHITE_LED, HIGH);
          mode = NORMAL;
          state = MOVE_TO_LOCK_WAIT;
          //count = 0;*/
            }
            digitalWrite(ledPin, LOW);    // set the LED off
            //pState = 0;    // wait for another press
          }
          break;
        case SETUP_HOME:
        // The correct algorithm would be: (subject to revision) lower to about n steps before stop. (previous step)
       //  Lower at some slow, constant speed until switch is hit, then stop. Set position to 0.
          digitalWrite(ledPin, HIGH);   // set the LED on
          stepper = allSteppers[rcount];
          stepper->setSpeed(FIND_HOME_SPEED);
          //stepper1.setAcceleration(1.0);
#if defined DEBUG
        Serial.println("in SETUP_HOME");
        Serial.print("rcount: "); Serial.println(rcount);
#endif
          state = FIND_HOME;
          break;
         case FIND_HOME:
          stepper->runSpeed();
          if (homeSensor[rcount] == LOW) {
            stepper->setCurrentPosition(0);
            stepper->stop();    // sets a new position
            stepper->runSpeedToPosition();    // runs to the new position   
            stepperToRelease[rcount]->release();      // Need to do this to keep motors from heating up
#if defined DEBUG
        Serial.println("in FIND_HOME - released ");
        Serial.print("rcount: "); Serial.println(rcount);
#endif
            if (rcount++ < STEPPER_COUNT){
              state = SETUP_HOME;
            }
            else {
              state = SETUP_RESET2;
              rcount = 0;
            }
            digitalWrite(ledPin, LOW);    // set the LED off
          }
          break;
        case SETUP_RESET2:      // now move stepper stuff to the start positions
          digitalWrite(ledPin, HIGH);   // set the LED on
          stepper = allSteppers[rcount];
          if (rcount == 0){      // Slower for the boat X position
            stepper->setMaxSpeed(BOAT_RESET_SPEED);
          }else{
            stepper->setMaxSpeed(RESET_SPEED);
          }
          stepper->setAcceleration(RESET_ACCEL);
          stepper->moveTo(startPosition[rcount]);
#if defined DEBUG
        Serial.println("in SETUP_RESET2 ");
        Serial.print("rcount: "); Serial.println(rcount);
#endif
          state = RESET_2;
          break;
        case RESET_2:
          stepper->run();
          if (stepper->distanceToGo() == 0) {
            stepperToRelease[rcount]->release();      // Need to do this to keep motors from heating up
#if defined DEBUG
        Serial.println("in RESET_2 released ");
        Serial.print("rcount: "); Serial.println(rcount);
#endif
            if (rcount++ < STEPPER_COUNT){
              state = SETUP_RESET2;
            }
            else {
              state = RESET_3;
            }
            digitalWrite(ledPin, LOW);    // set the LED off
            //pState = 0;    // wait for another press
          }
          break;
        case RESET_3:
          digitalWrite(ledPin, HIGH);   // set the LED on
#if defined DEBUG
        Serial.println("in RESET_3 ");
        Serial.print("rcount: "); Serial.println(rcount);
#endif
          for (rcount = 0; rcount < GATE_COUNT; rcount++) {
           target = gateReadyVal[rcount];
           val = analogRead(gatePositionSensor[rcount]);
           if (val > target){
             moveDown(gateMotor[rcount], gatePositionSensor[rcount], target);
           } else {
             moveUp(gateMotor[rcount], gatePositionSensor[rcount], target);
           }
          }
          digitalWrite(ledPin, LOW);    // set the LED off
          digitalWrite(RED_LED, HIGH);
          digitalWrite(GREEN_LED, HIGH);
          digitalWrite(WHITE_LED, HIGH);
          mode = NORMAL;
          state = MOVE_TO_LOCK_WAIT;
          count = 0;
          break;
      }
}

//--------------------------------------------------------------------------
// Move motor down to target position
void moveDown(Adafruit_DCMotor *theMotor, byte thePot, int posTarget)
{
  int theVal;
    theMotor->setSpeed(DNSPEED);
    theMotor->run(FORWARD);
    theVal = analogRead(thePot);
    while (theVal > posTarget){
       theVal = analogRead(thePot);
       delay (1);
    }
    theMotor->run(RELEASE);
}

//--------------------------------------------------------------------------
// Move motor up to target position
void moveUp(Adafruit_DCMotor *theMotor, byte thePot, int posTarget)
{
  int theVal;
    theMotor->setSpeed(UPSPEED);
    theMotor->run(BACKWARD);
    theVal = analogRead(thePot);
    while (theVal < posTarget){
       theVal = analogRead(thePot);
       delay (1);
    }
    theMotor->run(RELEASE);
}
/////////////////////////////////////////////////////////////////////////////
// Interrupt handler for WHITE button (selects AUTOMATIC or does RESET if GREEN is pressed also)
/////////////////////////////////////////////////////////////////////////////
void automaticButtonHandler()
{
  if ((mode != RESET) && (mode != RESETING)) {
    if (digitalRead(GREEN_BUTTON) == LOW){
      mode = RESET;
    } else if (mode != AUTOMATIC) {  // this really doesn't matter
      mode = AUTOMATIC;
    }
  }
}
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
