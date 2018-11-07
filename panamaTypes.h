//---------------------------------------------------------------------------------------
// Herein are defined the types and structures for PanamaCanal.
//  2/20/15    Initial Creation
#ifndef panamaTypes_h
#define panamaTypes_h

//------------------------------------------------------------------------------------------------
// Stepper Control Definitions
// NOTE: There is only one speed the steppers use during RESET.
//   May want to change this.
#define BOAT_SPEED  800.0
#define BOAT_ACCEL  800.0
#define WATER_SPEED  800.0
#define WATER_ACCEL  800.0
#define BOAT_RESET_SPEED  800.0
#define RESET_SPEED  300.0
#define RESET_ACCEL  200.0
#define FIND_HOME_SPEED 300.0
//------------------------------------------------------------------------------------------------
// Motor Control Definitions
// PWM speed is the frequency the PWM runs at. Lower PWM means the motors can run slower, but
//    running the motors slower can make them jerky and noisy.
// The up and down speeds don't need to be the same.
#define MOTOR_PWM_SPEED  100    
#define UPSPEED  80
#define DNSPEED  80
//------------------------------------------------------------------------------------------------
// Boat X positions in the locks. The boat moves about 128 steps per inch.
#define BOAT_X_HOME  0
#define LOCK1_WAIT_POSITION  120
#define LOCK1_INSIDE_POSITION  1000
#define LOCK2_WAIT_POSITION  1050
#define LOCK2_INSIDE_POSITION  1750
#define LOCK3_WAIT_POSITION  1850
#define LOCK3_INSIDE_POSITION  2350
#define LAGOON_WAIT_POSITION  2400
#define LAGOON_INSIDE_POSITION  2700
#define LOCK4_WAIT_POSITION  4540
#define LOCK4_INSIDE_POSITION  5390
#define LOCK5_WAIT_POSITION  5744
#define LOCK5_INSIDE_POSITION  6440
#define LOCK6_WAIT_POSITION  6600
#define LOCK6_INSIDE_POSITION  7500
#define OCEAN_WAIT_POSITION  7900
#define OCEAN_INSIDE_POSITION  8300

// Lock Water Levels and reference heights
// The HOME position is where the stepper goes to before starting the HOME sequence
//  (finding the endstop). Then the water can be raised to the MT positon.
// The REF position is the value of the boat Y-axis which corresponds to 0 of each
//  lock stepper.
#define BOAT_Y_HOME  0
#define LOCK1_MT  0
#define LOCK1_REF  0
#define LOCK1_HOME  0
#define LOCK1_FULL  6000
#define LOCK2_MT  0
#define LOCK2_REF  6000
#define LOCK2_HOME  0
#define LOCK2_FULL  6000
#define LOCK3_MT  0
#define LOCK3_REF  12000
#define LOCK3_HOME  0
#define LOCK3_FULL  6000
#define LOCK4_MT  0
#define LOCK4_REF  18000
#define LOCK4_HOME  0
#define LOCK4_FULL  6000
#define LOCK5_MT  0
#define LOCK5_REF  1200
#define LOCK5_HOME  0
#define LOCK5_FULL  6000
#define LOCK6_MT  0
#define LOCK6_REF  6000
#define LOCK6_HOME  0
#define LOCK6_FULL  6000
//------------------------------------------------------------------------------------------------
// Lock Gate Positions 
// These are for the DC motors that operate the gates. They are the ADC readings that corresponsd
//  to the desired positions.
#define GATE1_READY  200
#define GATE1_OPEN  100
#define GATE1_RAISED  900
#define GATE2_READY  800
#define GATE2_OPEN  900
#define GATE2_RAISED  100
#define GATE3_READY  400
#define GATE3_OPEN  300
#define GATE3_RAISED  900
#define GATE4_READY  200
#define GATE4_OPEN  100
#define GATE4_RAISED  900
#define GATE5_READY  100
#define GATE5_OPEN  100
#define GATE5_RAISED  900
#define GATE6_READY  200
#define GATE6_OPEN  100
#define GATE6_RAISED  900
#define GATE7_READY  200
#define GATE7_OPEN  100
#define GATE7_RAISED  900
#define GATE8_READY  200
#define GATE8_OPEN  100
#define GATE8_RAISED  500
//------------------------------------------------------------------------------------------------
// Gate Position Pot Channel Definitions
//   Mapping the port hookup to the gate number.
#define LOCK1_ENTER_GATE_POSITION_SENSOR  0
#define LOCK1_EXIT_GATE_POSITION_SENSOR  1
#define LOCK2_ENTER_GATE_POSITION_SENSOR  1
#define LOCK2_EXIT_GATE_POSITION_SENSOR  2
#define LOCK3_ENTER_GATE_POSITION_SENSOR  2
#define LOCK3_EXIT_GATE_POSITION_SENSOR  3
#define LAGOON_ENTER_GATE_POSITION_SENSOR  3

//#define LAGOON_EXIT_GATE_POSITION_SENSOR  4
//#define LOCK4_ENTER_GATE_POSITION_SENSOR  4
//#define LOCK4_EXIT_GATE_POSITION_SENSOR  5
//#define LOCK5_ENTER_GATE_POSITION_SENSOR  5

#define LAGOON_EXIT_GATE_POSITION_SENSOR  8
#define LOCK4_ENTER_GATE_POSITION_SENSOR  8
#define LOCK4_EXIT_GATE_POSITION_SENSOR  9
#define LOCK5_ENTER_GATE_POSITION_SENSOR  9

#define LOCK5_EXIT_GATE_POSITION_SENSOR  6
#define LOCK6_ENTER_GATE_POSITION_SENSOR  6
#define LOCK6_EXIT_GATE_POSITION_SENSOR  7
#define OCEAN_ENTER_GATE_POSITION_SENSOR  7
//------------------------------------------------------------------------------------------------
// Definition of States
// These control the movement of the boat through the locks. Don't change these
//  without careful thought!
#define BOAT_IDLE 0
#define MOVE_TO_LOCK_WAIT 1
#define MOVE_TO_LOCK_SETUP 2
#define MOVE_TO_LOCK 3
#define ENTER_LOCK_WAIT 4
#define ENTER_LOCK_SETUP 5
#define ENTER_LOCK 6
#define RAISE_LOCK_SETUP 7
#define RAISE_LOCK 8
#define PREP_LOWERING_LOCK_SETUP 9
#define PREP_LOWERING_LOCK 10
#define LOWER_LOCK_SETUP 11
#define LOWER_LOCK 12
#define SETUP_RESET 13
#define RESET_1 14
#define SETUP_HOME 15
#define FIND_HOME 16
#define SETUP_RESET2 17
#define RESET_2 18
#define RESET_3 19
//---------------------------------------------------------------------------------------------------
// Operating modes
// These control the movement of the boat through the locks. Don't change these
//  without careful thought!
#define NORMAL 0
#define AUTOMATIC 1
#define RESET  2
#define RESETING  3
//-------------------------------------------------------------------------------------------------
// Need 3 Buttons
// The BLUE button is known as RED inside the program. 
#define GREEN_BUTTON 14
#define RED_BUTTON 15
// the white button is tied to an interrupt
#define WHITE_BUTTON 2
//-------------------------------------------------------------------------------------------------
// Need 3 LEDs to light the buttons
// The BLUE LED is known as RED inside the program. 
#define GREEN_LED 17
#define RED_LED 18
#define WHITE_LED 19
//-------------------------------------------------------------------------------------------------
// Need 8 Home sensors
// These are the mapping of the stepper home switches to the ports
#define BOAT_HOME_SENSE_X 4
#define BOAT_HOME_SENSE_Y 5
#define LOCK1_HOME_SENSE 6
#define LOCK2_HOME_SENSE 7
#define LOCK3_HOME_SENSE 8
#define LOCK4_HOME_SENSE 9
#define LOCK5_HOME_SENSE 10
#define LOCK6_HOME_SENSE 11

//-------------------------------------------------------------------------------------------------
// Count test limits
// The limits that control the boat actions at defined points.
#define LAGOON_COUNT 3
#define OCEAN_COUNT 7
#define RESET_COUNT 7
//#define RESET_COUNT 7

#endif
