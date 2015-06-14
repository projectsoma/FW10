/** Three motor and shutter controller for Arduino Leonardo
This script was made to control three filter wheels and one shutter with a single Arduino Leonardo platform.
This script needs the libraries Accelstepper from http://www.airspayce.com/mikem/arduino/AccelStepper/ and Arduino Serial Command from https://github.com/kroimon/Arduino-SerialCommand . 
When sending string commands through serial, it needs the correct line terminator (\n) or choose the line ending option in the Arduino Command Monitor.
Avalaible functions and formats are:

POS F W: Moves wheel W to position F. Minimizes the amount of steps needed. W is an integer between 1 and 3. F is an integer between 1 and 5;
MOVE P W: Moves wheel W P positions. W is an integer between 1 and 3. P can be a positive or negative integer between -5 and 5;
SET1 W: Sets actual position of the wheel W as origin (1). W is an integer between 1 and 3;
POS? W: Returns a string stating the actual position of wheel W. W is an integer between 1 and 3;
STEP S W: Moves wheel W, S steps. W is an integer between 1 and 3;
SENS W: Senses the voltage returned by the sensor. W is an integer between 1 and 3;
RESET W: Resets the wheel W to the origin set by the marker and sets it as origin (1). W is an integer between 1 and 3. If the threshold can´t be found, a string is returned stating the problem. If it´s already at the marker, it moves away from it and searches for it again.
If an unmatching string is sent, a string reporting the error is returned.

Motors corresponding to wheels 1, 2 and 3 should be connected to Arduino pins (7,8), (9,10) and (11,12), respectively. Analogously, voltage sensing pins should be 0, 1 and 2, in the same order.
*/

// Import libraries
#include <SerialCommand.h>
#include <AccelStepper.h>

// Initiate variables
int Actual[3] = {1, 1, 1}; // Variable for actual position of each wheel
int pins[3][3]; // Variable for the Arduino pins where each motor is connected
int analogpin[3] = {0, 1, 2}; // Analog pin for the voltage measurement of the sensor
AccelStepper steppers[3]; // {AccelStepper(2, 7, 8), AccelStepper(2, 9, 10), AccelStepper(2, 11, 12)}; //set type motor, and both pins connected to Arduino
  
SerialCommand sCmd; // Rename command

// Setting system setup
void setup() {
  // Setting up the pin numbers for the pins variable
  for (int p = 0; p <= 2; p++) { 
    pins[0][0] = 2; // Set wire type
    pins[p][1] = 2 * p + 7; // pin
    pins[p][2] = 2 * p + 8; // pin
  }
  
  // Set shutter pin
  pins[3][3] = 1;
  
  // Setting the motor function pointer
  for (int p = 0; p <= 2; p++) {
    steppers[p] = AccelStepper(pins[p][0], pins[p][1], pins[p][2]);
  }
  
  //// Setup callbacks for SerialCommand commands
  sCmd.addCommand("POS", Position); // Moves to the specified position
  sCmd.addCommand("MOVE", Move); // Moves the amount of steps stated in the argument
  sCmd.addCommand("SET0", SetZero); // Sets actual Filter as zero
  sCmd.addCommand("POS?", StatePosition); // Ask for actual position
  sCmd.addCommand("STEP", Step); // Gives desired amount of steps
  sCmd.addCommand("SENS", ProximityVoltage); // Returns proximity sensor voltage
  sCmd.addCommand("RESET", Reset); // Resets wheel positioning
  //sCmd.addCommand("HELP", Help); // Prints Help text
  sCmd.setDefaultHandler(unrecognized); // Handler for command that isn't matched  (says "What?")
     
  // Motor characteristics:
  for (int m = 0; m <= 2; m++) {
    steppers[m].setMaxSpeed(10000);
    steppers[m].setAcceleration(37000);
  }
  
  // Start serial port communication with corresponding baud rate
  Serial.begin(9600);
  Serial.println("Ready");
}

// Serial polling loop
void loop() {
  while (Serial.available() > 0) {
    sCmd.readSerial();     // Process serial commands if new serial is available
  }
}

// Choose position function
void Position()
{
  // Initiating variables
  int Filter;
  int Wheel;
  char *arg;
  
  arg = sCmd.next();
  if (arg == NULL) {
    Serial.println("No position or wheel stated");
    return;
  }
  Filter = atoi(arg); // Setting first argument as the desired position
  arg = sCmd.next();
  if (arg == NULL) {
    Serial.println("No position or wheel stated");
    return;
  }
  Wheel = atoi(arg); 
  Wheel = Wheel - 1; // Choosing the desired wheel
 
 // Checking a valid position 
  if (Filter <= 0 || Filter >= 6) {
    Serial.println("Invalid Position");
    return;
  }
  
  // Checking a valid wheel 
  if (Wheel <= -1 || Wheel >= 3) {
    Serial.println("Invalid Wheel");
    return;
  }
  
  // Optimization of the steps needed
  int Difference = Filter - Actual[Wheel];
  
  if (Difference >= 3){
    MoveEffector (Difference - 5, Wheel); // Caller for move executer function
  }
  else if (Difference <= -3) {
    MoveEffector (5 + Difference, Wheel); // Caller for move executer function
  }
  else { 
    MoveEffector (Difference, Wheel); // Caller for move executer function
  }
}

// Moving a user specified amount of positions function
void Move() {
  // Variable initiation
  char *arg;
  arg = sCmd.next();
  if (arg == NULL) {
    Serial.println("No steps or wheel stated");
    return;
  }
  int Steps = atoi(arg); // Setting first argument as the desired amount of positions. Accepts negative steps
  arg = sCmd.next();
  if (arg == NULL) {
    Serial.println("No steps or wheel stated");
    return;
  }
  int Wheel = atoi(arg);
  Wheel = Wheel - 1; // Choosing the desired wheel
    
  // Checking a valid wheel 
  if (Wheel <= -1 || Wheel >= 3) {
    Serial.println("Invalid Wheel");
    return;
  }
  
  MoveEffector(Steps, Wheel); // Caller for move executer function
}
  
// Private function that actually executes the steps  
void MoveEffector(int Positions, int stepper)
{
   
  if (abs(Positions) <= 5) { // Checks number of positions to move so as not to turn all around
    int steps = 120 * Positions; // Calculates how many steps
    steppers[stepper].move(steps);
    while (steppers[stepper].distanceToGo() != 0){ // Doesn´t return control of Arduino until every step has been made
      steppers[stepper].run();
    }
  }
  
  else { // returns log if an excess of steps is asked for
    Serial.println("Step Excess");
    return;
  }
  // Recalculates actual position 
  if (Actual[stepper] + Positions > 5) { 
    Actual[stepper] = Actual[stepper] + Positions - 5;
  }
  else if (Actual[stepper] + Positions < 1) {
    Actual[stepper] = 5 + Actual[stepper] + Positions;
  }
  else {
    Actual[stepper] = Actual[stepper] + Positions;
  }
} 

// sets actual position to zero
void SetZero() { 
  // Variable initiation
  char *arg;
  arg = sCmd.next();
  // Checks argument found
  if (arg == NULL) {
    Serial.println("No wheel stated");
    return;
  }
  int Wheel = atoi(arg);
  Wheel = Wheel - 1; // Choosing the desired wheel
  
  // Checking a valid wheel 
  if (Wheel <= -1 || Wheel >= 3) {
    Serial.println("Invalid Wheel");
    return;
  }
  
  Actual[Wheel] = 1;
}

// Returns a string with the actual position
void StatePosition() {      
  // Variable initiation
  char *arg;
  arg = sCmd.next();
  // Checks argument found
  if (arg == NULL) {
    Serial.println("No wheel stated");
    return;
  }
  int Wheel = atoi(arg);
  Wheel = Wheel - 1; // Choosing the desired wheel
  
  // Checking a valid wheel 
  if (Wheel <= -1 || Wheel >= 3) {
    Serial.println("Invalid Wheel");
    return;
  }
  
  String Where = String(Actual[Wheel]); // Find the actual position of the wheel
  Serial.print("Position ");
  Serial.println(Where);
}

// Gives a the desired amount of steps
void Step() {
  // Variable initiation
  char *arg;
  arg = sCmd.next();
  // Checks argument found
  if (arg == NULL) {
    Serial.println("No amount of steps or wheel stated");
    return;
  }
  int Steps = atoi(arg);
  if (Steps >= 650) {
    Serial.println("Amount of steps equals more than a full turn");
    return;
  }
  
  arg = sCmd.next();
  // Checks argument found
  if (arg == NULL) {
    Serial.println("No wheel stated");
    return;
  }
  int Wheel = atoi(arg);
  Wheel = Wheel - 1; // Choosing the desired wheel
  
  // Checking a valid wheel 
  if (Wheel <= -1 || Wheel >= 3) {
    Serial.println("Invalid Wheel");
    return;
  }
    
  steppers[Wheel].move(Steps);
   while (steppers[Wheel].distanceToGo() != 0){ // Doesn´t return control of Arduino until every step has been made
     steppers[Wheel].run();
   }
}

// Returns the sensor voltage
void ProximityVoltage() {
  // Variable initiation
  char *arg;
  arg = sCmd.next();
  // Checks argument found
  if (arg == NULL) {
    Serial.println("No wheel stated");
    return;
  }
  int Wheel = atoi(arg);
  Wheel = Wheel - 1; // Choosing the desired wheel
  
  // Checking a valid wheel 
  if (Wheel <= -1 || Wheel >= 3) {
    Serial.println("Invalid Wheel");
    return;
  }
  
  float Voltage;
  int Sensor;
  Sensor = analogRead(analogpin[Wheel]); // Reads the voltage measured by Arduino. Returns a random voltage if disconnected
  Voltage = Sensor * (5.0/1023.0);
  Serial.println(Voltage,DEC);
}

// Moves the wheel to the original position and resets zero
void Reset() {
  // Variable initiation
  char *arg;
  arg = sCmd.next();
  float VoltageThreshold = 4.0;
  // Checks argument found
  if (arg == NULL) {
    Serial.println("No wheel stated");
    return;
  }
  int Wheel = atoi(arg);
  Wheel = Wheel - 1; // Choosing the desired wheel
  
  // Checking a valid wheel 
  if (Wheel <= -1 || Wheel >= 3) {
    Serial.println("Invalid Wheel");
    return;
  }
  
  int j = 0; // Low voltage step counter
  float Voltage; // Voltage calculated variable
  int Sensor; // Voltage sensed variable in bits
  
  // Calculate voltage sensed 
  Sensor = analogRead(analogpin[Wheel]);
  Voltage = Sensor * (5.0/1023.0);
  
  // Moves away from marker if it is already on it
  if (Voltage > VoltageThreshold) {
    MoveEffector(4, Wheel); // Caller for move executer function
  }
  
  // Calculate voltage sensed 
  Sensor = analogRead(analogpin[Wheel]);
  Voltage = Sensor * (5.0/1023.0);
  
  // Searchs for the beginning of the marker  
  while (Voltage <= VoltageThreshold && j <= 750) {
    steppers[Wheel].move(1);
    while (steppers[Wheel].distanceToGo() != 0){ // Doesn´t return control of Arduino until every step has been made
      steppers[Wheel].run();
    }
    // Calculate voltage sensed 
    Sensor = analogRead(analogpin[Wheel]);
    Voltage = Sensor * (5.0/1023.0);
    
    j = j + 1;
  }
  
  int i = 0; // High voltage step counter
  
  // Counts the steps to the end of the marker
  while (Voltage >= VoltageThreshold && i <= 750) {
    steppers[Wheel].move(1);
    while (steppers[Wheel].distanceToGo() != 0){ // Doesn´t return control of Arduino until every step has been made
      steppers[Wheel].run();
    }
    
    // Calculate voltage sensed 
    Sensor = analogRead(analogpin[Wheel]);
    Voltage = Sensor * (5.0/1023.0);
    i = i + 1;
  }
  
  // Returns log if a high voltage is measured in every step. Usually happens when the Arduino is disconnected or the marker can't be seen by the sensor
  if (i >= 749 || j >= 749) {
    Serial.println("Position marker not found");
    return;
  }
  
  // Returns to the middle of the points between both thresholds
  delay(300);
  steppers[Wheel].move(-i/2);
  while (steppers[Wheel].distanceToGo() != 0){ // Doesn´t return control of Arduino until every step has been made
    steppers[Wheel].run();
  }
  
  Actual[Wheel] = 1;
}

/**
// Prints Help text through serial
void Help() {
  Serial.println("Three motor and shutter controller for Arduino Leonardo");
  Serial.println("This script was made to control three filter wheels and one shutter with a single Arduino Leonardo platform.");
  Serial.println("This script needs the libraries Accelstepper from http://www.airspayce.com/mikem/arduino/AccelStepper/ and Arduino Serial Command from https://github.com/kroimon/Arduino-SerialCommand .");
  Serial.println("When sending string commands through serial, it needs the correct line terminator (\n) or choose the line ending option in the Arduino Command Monitor.");
  Serial.println("Avalaible functions and formats are:");

  Serial.println("POS F W: Moves wheel W to position F. Minimizes the amount of steps needed. W is an integer between 1 and 3. F is an integer between 1 and 5;");
  Serial.println("MOVE P W: Moves wheel W P positions. W is an integer between 1 and 3. P can be a positive or negative integer between -5 and 5;");
  Serial.println("SET1 W: Sets actual position of the wheel W as origin (1). W is an integer between 1 and 3;");
  Serial.println("POS? W: Returns a string stating the actual position of wheel W. W is an integer between 1 and 3;");
  Serial.println("STEP S W: Moves wheel W, S steps. W is an integer between 1 and 3;");
  Serial.println("SENS W: Senses the voltage returned by the sensor. W is an integer between 1 and 3;");
  Serial.println("RESET W: Resets the wheel W to the origin set by the marker and sets it as origin (1). W is an integer between 1 and 3. If the threshold can´t be found, a string is returned stating the problem. If it´s already at the marker, it moves away from it and searches for it again.");
  Serial.println("If an unmatching string is sent, a string reporting the error is returned.");

  Serial.println("Motors corresponding to wheels 1, 2 and 3 should be connected to Arduino pins (7,8), (9,10) and (11,12), respectively. Analogously, voltage sensing pins should be 0, 1 and 2, in the same order.");
}
*/

// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command) {
  Serial.println("Invalid Command");
}
