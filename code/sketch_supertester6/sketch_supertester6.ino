
// Arduino based debugging tool for a FIRST robot.
// Copywright FIRST Robotics Team 997 Corvallis Oregon 3/20/2013
// This code is provided free for use to any FIRST robotics team.  Feel free to modify or extend this code, but please let us
// know what you have done, and please keep a reference to our website in the code and the about menu.
// http://chsrobotics.org/

// This code leverages example code form many sources.  We have tried to include references
// in the relavent sections.  Hopefully we did not miss any.
//
// This code was developed to run on an ATmega328 microcontroller on a custom Printed Circuit Board.  For information on
// building your own boaed, or purchasing one from us, see our website above, or send an email message to 
// supertester997@gmail.com.
// However, since the inital development was done using an Arduino Uno with an LCD Shield, this code can also be run on an 
// Arduino with a few code modifications, and by adding the required connectors, switches, and a few additional components.  
// The following is one example of how this might be done, feel free to modify this connection format to suit your exact needs.
// This text-only description is relatively limited and for reference only.  If you want more detailed information
// such as a schematic or layout drawing for the design we used, please see our website.
//
// If you are attempting to run this code with an Arduino, you will need the main board, an LCD shield, and the following. 
//
// A source of mobile power.  Most likely a 9 volt battery and power switch connected to GROUND and VIN on the Arduino board.
// A 10K or similar linier potentiometer connected between power and ground and with the wiper contact going to analog input 0. 
// A variety of .1 inch 3 pin header connectors for interfacing to the commonly used PWM cables connected as follows.
// Two sets for output of PWM signals connected in the order of signal, power, ground, with the signal pins connected
// to digital output 3 and 10.  This set has the center pin powered from the boards 5 volts so that it can drive a servo.  
// Be careful, this circit can test a small servo, but does not have the power to drive a large servo, or even
// a small one if it is under heavy load.  Trying to drive something too large may cause a voltage drop which could 
// reset the circuit.  You do not need to connect the center pin to 5 volts if you will only be driving FIRST motor
// controllers, hoverver if you do that this tester will not be able to test servos.  

// Two sets of pins for use as two digital input ports. This is wired signal, power, ground with the signal pins being 
// connected to digital pins 1 and 2 from the Arduino.   These pins will be used for monitoring a PWM input, reading 
// encoders, and a few other things.  Digital pin 2 also corresponds to intrrupt 0 which will be needed for timing 
// of encoder pulses and PWM signals.  These inputs also need to connect to the outside pins of two additional three 
// pin connectors with the center pin grounded. 

// Two pulldown resistors (about 10K value) also need to be added between digital input pins 2 and ground, and between 
// digital pin 1 and ground.   The rsistors are required in place of the normal pull up resistors to allow testing of 
// active high SPIKE signals.  The Atmega chip does have built in pull up resistors that can be turned on in code to 
// avoid the need for adding physical resistors, but they are pull ups and not pull downs which would make it hard to 
// properly read SPIKE inputs.  If you do not plan to use the tester to check SPIKE signals you can change the code 
// to use internal pullups and not bother adding these resistors.

// Two sets for analog inputs connected in groups of signal power ground with the signal pins connected to analog 
// inputs 1, and 2.  Input 0, was already used to read the potentiometers, and input 3 is used as an extra input. 
// Analog inputs 4, and 5 are dedicated to the I2C connection.  One analog input is probably enough 
// for most uses, but I got carried away.
// By the way, the standard LCD shield uses Analog 0 for its buttons, so this mapping needs to be changed in the code
// to convert this code to work with the LCD shield.

// You also need to connnect two switches for use in controlling spike relays, to act as a dead man switch, and for other 
// functions.  These are connected to ground with a 5K ohm resistor.  

// If you are using an LCD shield for a display, it automatically makes the rest of the connections for you.  But you should be aware
// that this device uses digital pins 4,5,6,7,8,and 9, and analog pin 0, so these pins cannot be used for other things.  
// The display comes with several buttons, but this program currently only uses one of them, the next key.  If you mount your 
// tester in a box, be sure to provide access to this key.  

// That about all there is for hardware.  Team 997 is planning to build a printed circit board to make building one
// of these testers easier, or may decide to sell complete testers.  

// Finally this is the start of the real code.
// Include some libraries that will be used later:
#include <LiquidCrystal.h>    // Needed to drive our LCD display
#include <Servo.h>            // For outputting PWM signals
#include <Wire.h>             // For I2C communications.

// Define a few notes for use in the sonic screw driver sound effect
#define NOTE_A2  110
#define NOTE_G4  392
#define NOTE_F7  2794

Servo myservo1;               // create servo object to control a servo 
Servo myservo2;               // create second servo object to allow a second independent servo output 

// Define some constants to provide an easy place to remap the I/O or adjust some other variables.
// This should be handy if anyone develops custom hardware and wants to use this code.
const int PanelKnob = 0;      // analog pin used to read a potentiometer input that controls PWM
const int AnalogIO1 = 1;      // analog pin used to test various analog sensors, etc
const int AnalogIO2 = 2;      // analog pin used to read the scaled analog input (voltages above 5V)
const int NextButton = 13;    //  
const int SelectButton = 3;   //  
const int OptionButton1 = 0;  // 
const int OptionButton2 = 1;  // 
const int PWMOut1 = 3;        // 
const int PWMOut2 = 10;       // 
const int DigitalIO1 = 2;     // 
const int DigitalIO2 = 12;    // 
const int SpikeOut1 = 10;     // 
const int SpikeOut2 = 11;     //
const int Debounce = 500;     // The number of miliseconds to wait to prevent skipping multiple menus

// Define default I2C address for the ADXL345 accelerometer 
int devaddr = 0x1D;           // Stored as a variable so that it can be updated later using the scan option.

// Set up some variables to use for passing information to and from interrupt service routines
// These need to be of type volatile to insure they can pass data in and out of the service routine.
volatile int time_of_high_transition = 0;
volatile int time_of_low_transition = 0;
volatile int pulsewidth = 0;
volatile int time_between_pulses = 0;
volatile boolean rotationforward = false;
volatile boolean rotationlast = true;


// initialize the LCD library with the numbers of the interface pins
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
// And define some variables needed for this particular LCD shield
int ButtonVoltage = 0;
int ButtonPressed = 0;
int Backlight = 5;
int fadeValue = 255;

// Define registers for the ADXL345 accelerometer that is supported in I2C.
const char POWER_CTL = 0x2D;	//Power Control Register
const char DATA_FORMAT = 0x31;
const char DATAX0 = 0x32;	//X-Axis Data 0
const char DATAX1 = 0x33;	//X-Axis Data 1
const char DATAY0 = 0x34;	//Y-Axis Data 0
const char DATAY1 = 0x35;	//Y-Axis Data 1
const char DATAZ0 = 0x36;	//Z-Axis Data 0
const char DATAZ1 = 0x37;	//Z-Axis Data 1

byte _buff[6];                   // Buffer to read accelerometer data on I2C
                                 // should probably change code to pass this as a parameter sometime
// All this stuff is for the car ceash game.  Adapted from a game by @TheRealDod, Nov 25, 2010
// Converted the main code to a subroutine, removed the sound, and changed the I/O to match our hardware
const int RANDSEEDPIN = 0; // an analog pin that isn't connected to anything
const int MAXSTEPDURATION = 300; // Start slowly, each step is 1 millisec shorter.
const int MINSTEPDURATION = 150; // This is as fast as it gets
const int NGLYPHS = 6;
// the glyphs will be defined starting from 1 (not 0),
// to enable lcd.print() of null-terminated strings
byte glyphs[NGLYPHS][8] = {
  // 1: car up
  { B00000,
    B01110,
    B11111,
    B01010,
    B00000,
    B00000,
    B00000,
    B00000}
  // 2: car down
  ,{B00000,
    B00000,
    B00000,
    B00000,
    B01110,
    B11111,
    B01010,
    B00000}
  // 3: truck up
  ,{B00000,
    B11110,
    B11111,
    B01010,
    B00000,
    B00000,
    B00000,
    B00000}
  // 4: truck down
  ,{B00000,
    B00000,
    B00000,
    B00000,
    B11110,
    B11111,
    B01010,
    B00000}
  // 5: crash up
  ,{B10101,
    B01110,
    B01110,
    B10101,
    B00000,
    B00000,
    B00000,
    B00000}
  // 6: crash down
  ,{B00000,
    B00000,
    B00000,
    B10101,
    B01110,
    B01110,
    B10101,
    B00000}
};
 
const int NCARPOSITIONS = 4;
// Each position is mapped to a column of 2 glyphs
// Used to make sense when I had a 5th position
// where car or crash was drawn as 2 glyphs
// (can't do that since 0 terminates strings),
// so it's kinda silly now, but it ain't broke :)
const char BLANK=32;
char car2glyphs[NCARPOSITIONS][2] = {
  {1,BLANK},{2,BLANK},{BLANK,1},{BLANK,2}
};
char truck2glyphs[NCARPOSITIONS][2] = {
  {3,BLANK},{4,BLANK},{BLANK,3},{BLANK,4}
};
char crash2glyphs[NCARPOSITIONS][2] = {
  {5,BLANK},{6,BLANK},{BLANK,5},{BLANK,6}
}; 
const int ROADLEN = 15; // LCD width (not counting our car)
int road[ROADLEN]; // positions of other cars
char line_buff[2+ROADLEN]; // aux string for drawRoad()
int road_index;
int car_pos;
// Off-the-grid position means empty column, so MAXROADPOS
// determines the probability of a car in a column
// e.g. 3*NCARPOSITIONS gives p=1/3
const int MAXROADPOS = 3*NCARPOSITIONS;
int step_duration;
int crash; // true if crashed
unsigned int crashtime; // millis() when crashed
const int CRASHSOUNDDURATION = 250;
const char *INTRO1="Trucks ahead,";
const char *INTRO2="Drive carefully";
const int INTRODELAY = 2000;


// This is the tester setup function that runs at powerup.
// Any setup code that only needs to run once goes here.  This includes the splash screen which can be customized
// to show other team names or numbers.
void setup() {
  lcd.begin(16, 2);              // set LCD library for the number of cols and rows on display. 
  lcd.setCursor (0,0);           // set the cursor to column 0, line 0
  lcd.print("Team 997");         // Print a Flash Screen message to the LCD.  (May be customized as desired)
  lcd.setCursor (0,1);           // Position for the second line of text
  lcd.print("SuperTester 1.2");  // Print line 2 of the flash screen  (May be customized as desired)
  delay(1500);                   // Wait a short time to allow the Flash Screen to be read before running the main prog.

  // Initalize some inputs and outputs
  pinMode(0, INPUT);
  pinMode(1, INPUT);             // These will be used to read PWM and encoder inputs
  pinMode(2, INPUT);

  digitalWrite(OptionButton1, HIGH);      // set pullup on pin for front panel option button 1  
  digitalWrite(OptionButton2, HIGH);      // set pullup on pin for button 2 
  // digitalWrite(DigitalIO2, HIGH);        // set pullup on pin 12 
  digitalWrite(SelectButton, HIGH);       // set pullup on pin for the select button 
 
  pinMode(A0, INPUT);                     // Initalize the analog inputs
  pinMode(A1, INPUT);      
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  
  digitalWrite(NextButton, HIGH);         // set pullup for the NEXT button

  // Assign intrrupt 0 to cause a processer interrupt for any change on digital pin 2
  // This interrupt service routine (ISR) is called iservice which will be defined later.
  // An ISR is needed for features such as reading encoders and PWM inputs.
  attachInterrupt(0, iservice, CHANGE);    // Note, Interrupt 0 corresponds to digital pin 2,
  // Clear the screen before going into the main program
  Clearscreen();
}

// This is the main program loop which just cycles through each test then starts over.  It creates the tester
// menus and allows the user to either skip a function or to run it depending on what buttom they press.
// To make this method of selecting and running individual tests complete, each individual test function should 
// be set up to run continuously until the "NEXT" key is hit.    
void loop() {                              // This loop creats all of the top level menus
  if (Waitforinput("PWM") )                // Call a function to display a menu and wait for user input
      MyPWMOUT();                          // Then run or skip that function depending on the user input
  if ( Waitforinput("Spike") )             // Display the next menu choise
    SpikeTests();                          // Run it or not depending on user input
  if (Waitforinput("Encoder") )            // And so on.
    EncoderTests();
  if ( Waitforinput("Digital") )
    Myswitch();
  if ( Waitforinput("Analog") )
    Myanalog();
  if ( Waitforinput("I2C") )
    I2CTests();
  if ( Waitforinput("Utilities") )
    Utilities();
  if ( Waitforinput("About") )
    About();
}                                          // When you pass the last menu, start over from the begining.

// The tester has so many functions now that it can become tedious to look for the function you want,
// so it is helpful to organize some similar functions into secondary menus.  To do that just create 
// another function similar to the main loop with a collection of similar functions, and then call 
// that function from the main loop rather than calling one of the indvidual tests directly.
// This allows adding extra features without cluttering up the main menu as much.
void I2CTests()                         // This function creates the I2C sub menu.
{
  topMenu("I2C");                      // Display the main menue on line one.
  if ( WaitforinputSub("Scanner") )     // Then call the wait for input finction that does the rest
    I2CScan();                          // But only run the function if Waitforinput returns a TRUE
  topMenu("I2C");                      // Etc.
  if ( WaitforinputSub("ADXL345") )
    Accell();
  topMenu("I2C");
  if ( WaitforinputSub("Accel Fun") )
    AccellFun();
  topMenu("I2C");
  if (WaitforinputSub("other") )
    Dummycode(); 
}

void EncoderTests()                           // This function creates the encoder tests sub menu.
{
  topMenu("Encoder");                        // Refresh the top level menu
  if (WaitforinputSub("monitor") )            // Wait for user input
    MyEncoder();                              // Execute the function based on user input
  topMenu("Encoder");
  if (WaitforinputSub("Speed Control") )      // Lather. Rinse. Repeat 
    MySpeedControl();
  topMenu("Encoder");
  if (WaitforinputSub("Simulator") )
    MyEncoderOut();
  topMenu("Encoder");
  if (WaitforinputSub("One channel out") )
    PulsesOnly();
  // Add more Encoder tests here
}

void SpikeTests()                             // This function creates the Spike controls sub menu.
{
  topMenu("Spike");
  if (WaitforinputSub("Spike In Out") )
    MySpikeInOut();
   topMenu("Spike");
   if (WaitforinputSub("Spike Switch Ctl") )
    Mycompress();
}

void Utilities()                             // This function creates the utilities sub menu.
{
  topMenu("Utility");
  if (WaitforinputSub("Logic Analizer") )
    LogicAnalizer();
  topMenu("Utility");
  if (WaitforinputSub("Freq Counter") )
    MyPulseCount();
    topMenu("Utility");
  if (WaitforinputSub("Drive train") )
    DriveTrain();
  topMenu("Utility");
  if (WaitforinputSub("SelfTest") )
    SelfTest();
   topMenu("Utility");
  if (WaitforinputSub("Stopwatch") )
    StopWatch();
    topMenu("Utility");
  if (WaitforinputSub("Gyro") )   
    Mygyro();
    topMenu("Utility");
  if (WaitforinputSub("SonicScrewdriver") )   
    Sonic();
   topMenu("Utility");
  if (WaitforinputSub("Extra Function") )
    Dummycode();
}
// Any more sub menues should go here.

// This is an interrupt sevice routine for reading transitions on digital input pin 2.
// This interrupt will be used for timing PWM input signals and for reading encoder signals.
// Interrupt service routines can cause performace problems if they take too long to run.
// Use assignments, and limit calculations to keep the service routine as short and fast as possible.
// Note,  This interrupt service routine may be too long and slow for reliable measurements in some cases.
// You may want to change this routine to get more reliable speed and direction information for encoders
// running at high speeds, or other high speed inputs.
void iservice()                           // Interrupt service routine for decoding PWM, encoder signals, or other periodic inputs
{  
  if (digitalRead(DigitalIO1) == HIGH)             // This must have been a rising edge
  {
    if (digitalRead(DigitalIO2) == HIGH)          // If this is an encoder then rotation direction is reverse
    {
      rotationforward = false;
    }
    else
    {
      rotationforward = true;
    }
    time_between_pulses = (micros() - time_of_low_transition);
    time_of_high_transition = micros();   // Save this transition time for timing the pulsewidth
   } 
  else                                    // This was a falling edge
  {
    pulsewidth = (micros() - time_of_high_transition);
    time_of_low_transition = micros();    // save the time for measuring time between pulses
  }
}    


// Warning.  Be careful with strings.  there is only 2K of RAM and strings are stored in RAM.  If it is used up,
// the tester can act erraticlly.  These following functions help reduce the need for storing lots of blanks
// in strings by providing an easy way to clear various areas of the display.
// First, a function used to clear a defined number of spaces of the LCD screen from curent curser position.
void Clearspace(int space){    // Clear "space" number of positions
  for (int Location = 0; Location <= space; Location++) { 
  lcd.print(" ");    // Print blanks
  }
}

// A function used to clear a full line of the LCD screen.  You can pass any integer to this function, but
// since the tester uses a two line display, only  Clearline(0);, and Clearline(1); would make any sense.
void Clearline(int line){              // Pass the line number to clear
  lcd.setCursor (0,line);              // Start at the first space 
  Clearspace(16);                      // Print a bunch of blanks
}                                      // and return

// A simple function used to clear both lines of the LCD screen
void Clearscreen(){
  Clearline(0);
  Clearline(1);
}

// This function is the basis of the user interface.  It allows you to pass a string to use as a menu title.
// It displays the menu, and then enters a loop and waits for user input.  Depending on the input, it returns 
// with a value of either true or false.  This makes it easy to use this function with an IF statement to  
// either run or skip individual tester functions.
boolean Waitforinput(char* chars) 
{
  Clearscreen();
  lcd.setCursor (0,0);                           // Position for writing to screen
  lcd.print(chars);                              // Label what function we are on
  delay(Debounce);                               // Wait a short time to reduce chance of skipping commands
  ButtonVoltage = analogRead(SelectButton);      // read the button input value that will be used to exit this fucntion
  while ( (ButtonVoltage > 500) && (digitalRead(NextButton) == HIGH) )  
  {
    ButtonVoltage = analogRead(SelectButton);    // read the button input value that will be used to exit this fucntion
  } 
  return ExitWait();                             // Exit the function and return a value based on what button was hit
}


// This is a copy of the Waitforinput function, but modified to work with sub menus.  There is probably a much more
// elegant way to combine this with the Waitforinput function, but for now this is a quick way to impliment sub menus.  
boolean WaitforinputSub(char* chars) 
{
  lcd.setCursor (0,1);                           // Position for writing to screen
  lcd.print(chars);                              // Label what function we are on
  Clearspace(10);                                // Make sure the rest of the line is empty
  delay(Debounce);                               // Wait a short time to reduce chance of skipping commands
  ButtonVoltage = analogRead(SelectButton);      // read the button input value that will be used to exit this fucntion
  while ( (ButtonVoltage > 500) && (digitalRead(NextButton) == HIGH) )  
  {
    ButtonVoltage = analogRead(SelectButton);    // read the button input value that will be used to exit this fucntion
  }  
  return ExitWait();                             // Exit the function and return a value based on what button was hit
}


// A small function to help simplify the various wait for input functions.
// It clears the screen, reads what button was pressed, and then decides what value to return
// to tell the calling function if the displayed menu choice should be executed or skipped.
boolean ExitWait()                       // Small function to help simplify the wait for input functions
{ 
  Clearscreen();
  lcd.setCursor (0,0);

  if (digitalRead(NextButton) == HIGH)  // If the Select button was hit, return true
  {
    return true;
  }
  else                                  // Otherwise NEXT was hit so return false 
  {
    return false;
  }
}

// This function is used to help setup the display for showing a submenu.  This portion only clears 
// the screen and displays the top level menu.  When used with Waitforinputsub the display should show
// the top level menu on the first line, and a submenu on the second line.
void topMenu(char* chars) 
{
  Clearscreen();
  lcd.setCursor (0,0);                // Position for writing to screen
  lcd.print(chars);                   // Label what function we are on
}


// This function is used to read the potentiometer when you want +/- control around a center off position.
// It is designed to return a scaled potentiometer input while adding a slight dead band in the middle.
// This should give the tester a larger and more stable center off position.
// It returns the potentiometer reading converted to a scale of -1000 to 1000 with a slight dead band around 0.
// You pass an analog port number to the function so that it can also be used with external potentiometers.
// Recently increased the deadband to +- 200 to allow better zeroing with cheap potentiometers.  This could be
// reduced depending on the tolerence of the parts used to build the tester. 
int ReadPot(int AlogPort) 
{
  int ain;                                        // a variable to store the analog input value
  ain = analogRead(AlogPort);                     // reads the value of the potentiometer (value between 0 and 1023) 
  ain = map(ain, 0, 1023, -1200, 1200);           // scale it to +/- 1200
   if ((ain <= 200 ) && (ain >= -200))  ain = 0;  // Remove any values +/- 200 around the center position
   else if (ain > 200)  ain = ain - 200;          // Suntract 100 from the magnitude in both directions
   else if (ain < -200)  ain = ain + 200;         // This should leave the output approx +/- 1000
   if (ain > 1000)  ain = 1000;                   // Ensure that the result is never more than +/- 1000
   else if (ain < -1000)  ain = -1000;  
   return ain;
}


// Reads analog port 2 and returns +/- 1000 with a deadband and scaled to 5V
// Right now the second external port is configured with a voltage divisor to allow it to work with voltages
// above 5 volts.  This function converts the output so that it reads as if it were a normal analog port and
// then scales it just like the ReadPot function.
int ReadA2As5V() 
{
  int ain;                                           // a variable to store the analog input value
  ain = analogRead(AnalogIO2);                       // reads the value of the potentiometer (value between 0 and 1023) 
  ain = map(ain, 0, 1023, -1100, 58000);             // scale it to +/- 110
  if ((ain <= 100 ) && (ain >= -100))  ain = 0;      // Remove any values +/- 100 around the center position
  else if (ain > 100)  ain = ain - 100;              // Suntract 100 from the magnitude in both directions
  else if (ain < -100)  ain = ain + 100;             // This should leave the output approx +/- 1000
  if (ain > 1000)  ain = 1000;                       // Ensure that the result is never more than +/- 1000
  return ain;
}  

// A fuction that checks the knob input to see if it was centered.  If not, it tells the user to center
// the knob and hit next.  It can be handy if you want to force the user to start from a null output to the 
// motor controllers.
void ZeroKnob(int AnPort) 
{
  int ain;                                  // a variable to store the analog input value
  ain = ReadPot(AnPort);                    // reads the value of the potentiometer (value between 0 and 1023) 
  if (ain != 0)
  {
    while ((ain != 0)  ||  (  (digitalRead(NextButton) == HIGH) )) {  
      lcd.setCursor (0,0);                  // Set up the labels on screen.
      lcd.print ("Center knob then");
      lcd.setCursor (0,1);
      lcd.print ("hit Next "); 
      printjustify(ain);
      lcd.print ("  ");
      ain = ReadPot(AnPort);                // reads the value of the potentiometer (value between 0 and 1023) 
  }
 }  
 delay(Debounce);                           // Wait a short time to reduce the chance you skip multiple states
}

//  This function is similar the zeroknob function, but for use with two external potentiometers.
void ZeroPots() 
{
  int ain;                             // a variable to store the analog input value
  int ain2;                            // a variable to store another analog input value
  ain = ReadPot(AnalogIO1);            // reads the value of the potentiometer (value between 0 and 1023) 
  ain2 = ReadA2As5V();                 // reads the value of the potentiometer (value between 0 and 1023) 
  if ((ain != 0) || (ain2 != 0))
  {
    while ((ain != 0)  || (ain2 != 0) ||  (  (digitalRead(NextButton) == HIGH) )) {  
      lcd.setCursor (0,0);            // Set up the labels on screen.
      lcd.print ("Zero Pots & Next");
      lcd.setCursor (0,1);
        ain = ReadPot(AnalogIO1);                // reads the value of the potentiometer (value between 0 and 1023) 
        ain = map(ain, -1000, 1000, -100, 100);  // scale it to +/- 110 
        printjustify(ain);
        Clearspace(1);
        
        ain2 = ReadA2As5V();                     // reads the value of the potentiometer (value between 0 and 1023) 
      ain2 = map(ain2, -1000, 1000, -100, 100);           // scale it to +/- 110
      lcd.setCursor (7,1);                     // Set up the labels on screen.
      printjustify(ain2);    
      Clearspace(1);
    }
  }  
 delay(Debounce);                              // Wait a short time to reduce the chance you skip multiple states
}


// Read an analog port (passed parameter) and return the reading scaled for use as a PWM output
int Servoscale(int alogport) 
{ 
    int ain;                                       // a variable to store the analog input value
    int pwmout;                                    // a variable to store the planned output PWM value 
    pwmout = map(ReadPot(alogport), -1000, 1000, 0, 180);  // Read and scale analog input to a value between 0 and 180 for PWM output (Not 179 to gaurantee full scale)
    if (pwmout > 179) pwmout = 179;                // used for PWM output to account for potentiometers sometimes not reading full scale 
    return pwmout;                                 // Return the current output valued for displaying on the screen
}


// This function initalizes some I/O pins for use as servo outputs.
void SetupServos() 
{   
  pinMode(PWMOut1, OUTPUT);  
  pinMode(PWMOut2, OUTPUT);    
  myservo1.attach(PWMOut1);        // attaches pin identified as PWMOut1 to the first servo object 
  myservo2.attach(PWMOut2);        // attaches pin identified as PWMOut2 to the second servo object   
  myservo1.write(90);              // Make sure that PWM outputs start with motors turned off 
  myservo2.write(90);              // Make sure that PWM outputs start with motors turned off 
}

// This function releases the I/O ports from use as servo outputs.
void ClearServos() 
{   
  myservo1.write(90);                     // turn off motor controllers when this routine exits.  Just to be sure.
  myservo2.write(90);                     // turn off motor controllers when this routine exits. 
  myservo1.detach();                      // detach servo object so that this pin can be used for other things 
  myservo2.detach();                      // detach servo object so that this pin can be used for other things
 
}

// This function is designed to read the potentiometer and then output PWM control signals on two seperate
// I/O ports.  One output is in the reverse direction of the other since the positive rotation direction for any motor   
// is aribtrary and is based on the specific gearing and wiring on each robot.  Having two seperate outputs that
// are the inverse of each other can be handy particularly when trying to motor motion to match encoder readings.
int DualServoOut() 
{ 
    int pwmout;                                    // a variable to store the planned output PWM value 
    pinMode(PWMOut1, OUTPUT);                      // Initalize the servo outputs
    pinMode(PWMOut2, OUTPUT);                      // Initalize the servo outputs  
    pwmout = Servoscale(PanelKnob);                // Read analog input to a value 
    pwmout = map(pwmout, 179, 0, 0, 185);          // Reverse the direction of pwmout    
    if (digitalRead(OptionButton1) == HIGH) {      // If the user is not holding the dead man switch
     pwmout=90;                                    // Set the motor output to zero to prevent motion.
    lcd.setCursor (0,0); 
    lcd.print("Hold F1"); 
    Clearspace(6);
    delay(500);
     }
    myservo2.write(pwmout);                        // Outputs a copy of the servo control signal in reverse on a seperate I/0
    pwmout = Servoscale(PanelKnob);                // Read analog input to a value 
    if (digitalRead(OptionButton1) == HIGH)        // If the user is not holding the dead man switch
     pwmout=90;                                    // Set the motor output to zero to prevent motion.
    myservo1.write(pwmout);                        // Outputs the servo control according to the scaled value 
    return pwmout;                                 // Return the current output valued for displaying on the screen
}

// A fuction to test the front panel knob and buttons.
void SelfTest() 
{
int ain;
while (true) {  
  int ain;                                           // a variable to store the analog input value
  ain = map(ReadPot(PanelKnob), -1000, 1000, 0, 16); // Read and scale the analog input from 0 to 16
  lcd.setCursor (0,0);
  for (int Location = 0; Location <= ain; Location++) {
    lcd.print ("*");
  }
  Clearspace(17-ain);
  lcd.setCursor (0,1);
  if (analogRead(SelectButton) <= 500)              // If the user is pressing select switch
  lcd.print ("Select");                             // Display the value
  if (analogRead(SelectButton) > 500)               // If the user is not pressing select switch
  Clearspace(6);
  lcd.setCursor (6,1);
  if (digitalRead(OptionButton1) == LOW)            // If the user is pressing F1
  lcd.print ("F1");
  if (digitalRead(OptionButton1) == HIGH)           // If the user is not pressing F1
  Clearspace(3); 
  lcd.setCursor (9,1);
  if (digitalRead(OptionButton2) == LOW)            // If the user is pressing F2
  lcd.print ("F2");
  if (digitalRead(OptionButton2) == HIGH)           // If the user is not pressing F2
  Clearspace(3);
  lcd.setCursor (12,1);
  if (digitalRead(NextButton) == LOW)               // If the user is pressing Next
  lcd.print ("Next");
  if (digitalRead(NextButton) == HIGH)              // If the user is not pressing Next
  Clearspace(6);
  delay(50);
  }
 delay(Debounce);                                   // Wait a short time to reduce the chance you skip multiple states
}

// A function to output two PWM signals based on the input from two external potentiometers.
void DriveTrain(){   
  int pwmsent;                                 // Variable to store the value that was output to the PWM port
  int pwmin;                                   // Variable used to decode any PWM signal that was read.
  ZeroPots();                                  // Make sure the inputs are not set to extreme values.
  lcd.setCursor (0,0);                         // Set up the labels on screen.
  lcd.print ("One Side");                      // Label one input
  Clearspace(4);                     
  lcd.setCursor (0,1);
  lcd.print ("Other Side");                    // And the other
  Clearspace(1); 
  SetupServos();                               // Setup two I/Os for use as servo outputs 
  while (digitalRead(NextButton) == HIGH) {    // Run this routine until the "NEXT" key is hit 
    DriveTrainOut();                           // Outputs two servo signals 
    delay(100);                                // waits for device to respond, may also keep display from being too jumpy
  } 
  Clearscreen();                               // When done clear the screen
  ClearServos();                               // Free up I/O for other uses.
  delay(500);                                  // Wait a short time to reduce the chance you skip multiple states
  return;
} 

// Similar to DualServoOut, but it outputs two independent PWM signals based on the external analog ports.
// It also used an external dead man switch, does not return the PWM values, and does not reverse the second output.
void DriveTrainOut() 
{ 
    int pwmout;                                    // a variable to store the planned output PWM value 
    pinMode(PWMOut1, OUTPUT);                      // Initalize the servo outputs
    pinMode(PWMOut2, OUTPUT);                      // Initalize the servo outputs
    pwmout = Servoscale(AnalogIO1);                // Read and scale analog input to a value between 0 and 179 
    if (digitalRead(DigitalIO1) == LOW)            // If the user is not holding the dead man switch
       pwmout=90;                                  // Set the motor output to zero to prevent motion.
    myservo1.write(pwmout); 
    lcd.setCursor (11,0);
    pwmout = map(pwmout, 0, 179, -200, 220);    // scale analog output to +/- 100%
    printjustify(pwmout); 
    Clearspace(2);
     // Outputs the servo control according to the scaled value 
    pwmout = map(ReadA2As5V(), -1000, 1000, 0, 179);  // Read and scale analog input to a value between 0 and 179 for PWM output
      if (digitalRead(DigitalIO1) == LOW)         // If the user is not holding the dead man switch
    pwmout=90;                                     // Set the motor output to zero to prevent motion.
    myservo2.write(pwmout);                        // Outputs a copy of the servo control signal in reverse on a seperate I/0 
    lcd.setCursor (11,1);
    pwmout = map(pwmout, 0, 179, -220, 220);    // scale analog output to +/- 100%
    printjustify(pwmout); 
    Clearspace(2);
    myservo1.write(pwmout);  
    return;                                        // Return the current output valued for displaying on the screen
}


// A very basic logic analizer function.  It is not clear if this has enough speed or resolution to be valuable.
void LogicAnalizer() 
{ 
 int  PauseTime;  
 boolean ran;
 while (digitalRead(NextButton) == HIGH) {    // Run this routine until the "NEXT" key is hit 
  lcd.setCursor (0,0); 
  lcd.print("Wait for trigger"); 
  while (digitalRead(DigitalIO1) == HIGH) {   // Wait till digital input 1 is low
    showscan();
  }
  while (digitalRead(DigitalIO1) == LOW) {    // Now wait till it is high again
    showscan();
      // Should work like a simple trigger
  } 
  Clearscreen();
  PauseTime = map(ReadPot(PanelKnob), -1000, 1000, 2000, 5);ReadPot(PanelKnob); 
  for (int Location = 0; Location <= 15; Location++) { 
    showscan();
    lcd.setCursor (Location,0);                // Position 
      if (digitalRead(DigitalIO1) == HIGH)
        lcd.print("-");
      if (digitalRead(DigitalIO1) == LOW)  
        lcd.print("_");  
    lcd.setCursor (Location,1);                // Position 
      if (digitalRead(DigitalIO2) == HIGH)
        lcd.print("-"); 
        if (digitalRead(DigitalIO2) == LOW)
        lcd.print("_");   
      delayMicroseconds(PauseTime*10);        
    } 
    delay(1000);
    ran = false;  
    if ((digitalRead(OptionButton2) == LOW)&&(ran==false) ){
      lcd.setCursor (15,1);                // Position for writing to screen
      lcd.print( "P");
      delay (Debounce);
      while (digitalRead(OptionButton2) == HIGH) {   
      }
      ran=true;
    }
  }
}


// Used with logic analizer function to pop up the current scan rate on the screen
// converts Pausetime into millisecond and microsecond notation and displays in corner
// whenever Options Button 1 is held.
void showscan(){
  int PauseTime;
       while (digitalRead(OptionButton1) == LOW) {
        lcd.setCursor (9,1);                // Position for writing to screen
        PauseTime = map(ReadPot(PanelKnob), -1000, 1000, 1, 2000);ReadPot(PanelKnob); 
        lcd.print(PauseTime/100);
        lcd.print( "ms/");
        if ((PauseTime) <= 100) {
          lcd.setCursor (9,1);                // Position for writing to screen
          lcd.print(PauseTime*10);
          lcd.print( "Us/");
          delay(500);
        }  
      } 
}

// A routine to right justify numbers when they are displayed
// This feature is not yet used universally by all functions
void printjustify(int n) 
{
  if (abs(n)<1000) {
    lcd.print(" ");
    if (abs(n)<100) {
      lcd.print(" ");
      if (abs(n)<10) {
        lcd.print(" ");
        if (abs(n)<0) {
          lcd.print(" ");
        }  
      }
    }
  }
  lcd.print(n);
}

// This subroutine is designed to output a two PWM signals on two I/O pins.  
// However, one output is the inverse of the other.  If you care about the relationship between the knob and the output
// you can pick the appropriate output.  If you don't, you do not have to worry which output you use.
// This routine can also read and decode one PWM input on the digital input port.
void MyPWMOUT(){   
  int pwmsent;                             // Variable to store the value that was output to the PWM port
  int pwmin;                               // Variable used to decode any PWM signal that was read.
  ZeroKnob(PanelKnob);                     // make sure that the control knob is centered before starting output
  lcd.setCursor (0,0);                     // Set up the labels on screen.
 // lcd.print ("PWMOut ");
  lcd.setCursor (0,1);
//  lcd.print ("PWMIn "); 
  SetupServos();                               // Setup two I/Os for use as servo outputs 
  while (digitalRead(NextButton) == HIGH) {    // Run this routine until the "NEXT" key is hit 
    pwmsent = DualServoOut();                  // Outputs out two servo signals one the inverse of the other
    if (digitalRead(OptionButton1) == LOW) {   // If the user is holding the dead man switch
      lcd.setCursor (0,0);
      lcd.print ("PWMOut ");                     // Label the output
    }
    if (pwmsent > 90)  lcd.print (" ");        // Adjust location to account for minus sign 
    pwmsent = map(pwmsent, 0, 179, -220, 220); // scale the output to read -100 % to 100% 
    printjustify(pwmsent);
    lcd.print ("%   ");  
    lcd.setCursor (0,1);
    lcd.print ("PWMIn ");                      // The folowing portion of code looks for PWM input
    if ((time_between_pulses < 8000) || (time_between_pulses > 21000) )
    {
    lcd.print ("Not found");                // If the timing look wrong, say that the input was not detected
    }
    else
    {
      pwmin = map(pulsewidth, 550, 2390, -220, 220);       // scale input to read in +/- percent 
      printjustify(pwmin);
      lcd.print ("%   ");
      time_between_pulses = 0;            // Zero the input values to insure we are not seeing stale values
      pulsewidth = 0;
    }
    delay(100);                           // waits for device to respond, may also keep display from being too jumpy
  } 
  Clearscreen();                          // When done clear the screen
  ClearServos();                          // Free up I/O for other uses.
  delay(Debounce);                        // Wait a short time to reduce the chance you skip multiple states
  return;
} 

// This function generates a quadrature encoder output with speed and direction proportional to the control knob.
void MyEncoderOut(){ 
  int ain;
  int encoderstate = 1;                           // Variable to keep track of what encoder state we are currently simulating
  lcd.setCursor (0,0);
  lcd.print ("Encoder Simulatr");                 // Display what function we are running
  lcd.setCursor (0,1);
  lcd.print ("Running");
  pinMode(DigitalIO1, OUTPUT);                    // Initalize the digitak pins as outputs for this function
  pinMode(DigitalIO2, OUTPUT);                    // Since they are normally used as inputs
  while (digitalRead(NextButton) == HIGH) {       // Run this routine until the "NEXT" key is hit
    ain = ReadPot(PanelKnob);                     // reads and scales the value of the potentiometer (value between +/- 1000) 
    if (ain == 0)                                 // If the control is in the dead band
    {
      lcd.setCursor (0,1);
      lcd.print ("Stopped ");
    }
    if (ain > 0)                                  // If the control is above the dead band
    {
      encoderstate = encoderstate +1;             // Step the state ahead one position
      lcd.setCursor (0,1);
      lcd.print ("Forward ");
      printjustify(ain);
      Clearspace(4);
    }
    if (ain < 0)                                 // If the control is below the dead band
    {
      encoderstate = encoderstate -1;            // Step the state back one position
      lcd.setCursor (0,1);
      lcd.print ("Reverse ");
      printjustify(ain);
      Clearspace(4);
    }

    if (encoderstate == 5)                       // Correct the state for wrap around
      encoderstate = 1;
    if (encoderstate == 0)
      encoderstate = 4;

    if (encoderstate == 1)                       // Then output the new state to I/O pins
    {
      digitalWrite(2, HIGH); 
      digitalWrite(12, HIGH);
    }
    if (encoderstate == 2)
    {
      digitalWrite(2, HIGH); 
      digitalWrite(12, LOW);
    }
    if (encoderstate == 3)
    {
      digitalWrite(2, LOW); 
      digitalWrite(12, LOW);
    }
    if (encoderstate == 4)
    {
      digitalWrite(2, LOW);  
      digitalWrite(12, HIGH);
    }
    if (ain < 0) 
      ain = -ain;
    delay((1000-ain)/10);                        //  Wait until the next next pulse should be output
  } 
  Clearscreen();                                 // When done clear the screen
  pinMode(DigitalIO1, INPUT);                    // Return pins 2 and 12 to their normal state 
  pinMode(DigitalIO2, INPUT);                    // as inputs
 
  delay(Debounce);                               // Wait a short time to reduce the chance you skip multiple states
  return;
} 


// A faster encoder emulator for when only one channel is needed.
void PulsesOnly(){                           
  int ain;                                    // a variable to store the analog input value
  int freq;                                   // a variable to store the calculated frequency to output
  int encoderstate = 1;                       // State counter
  lcd.setCursor (0,0);
  lcd.print ("Square wave out");              // Label what we are doing
  Clearline(1);
  while (digitalRead(NextButton) == HIGH) {   // Run this routine until the "NEXT" key is hit
    ain = analogRead(PanelKnob);              // reads the value of the potentiometer (value between 0 and 1023) 
    freq = map(ain, 0, 1023, 50, 5000);       // scale the analog input to a frequency  
    lcd.setCursor (0,1);
    lcd.print (freq);
    lcd.print (" HZ  ");
    tone(2, freq);                            // Use the tone fuction to produce pulses
  } 
  Clearscreen();                              // When done clear the screen
  noTone(DigitalIO1);                         // Stop the output

  delay(Debounce);                            // Wait a short time to reduce the chance you skip multiple states
  return;
} 


// This routine is designed to read encoder inputs and display the direction and relative speed.
// Only one interrupt is used so it is not always accurate, but then again it needs to be compact. 
// It also outputs a PWM waveform for testing convienience.
void MyEncoder(){ 
  int pwmsent;
  ZeroKnob(PanelKnob);                           // make sure that the control knob is centered before starting output
  Clearline(0);
  Clearline(1);
  SetupServos();                                 // Set up two servo outputs
  while (digitalRead(NextButton) == HIGH) {      // Run this routine until the "NEXT" key is hit
    pwmsent = DualServoOut();                    // Outputs out two servo signals one the inverse of the other
    lcd.setCursor (0,0);
    if (digitalRead(OptionButton1) == LOW){      // Only label the output if F1 is held
      lcd.print ("Encoder PWM");                 // Label the output
      if (pwmsent > 90)  lcd.print (" ");        // Adjust location to account for minus sign 
      pwmsent = map(pwmsent, 0, 179, -220, 220); // scale the output to read -100 % to 100% 
      printjustify(pwmsent);
      lcd.print ("%   ");  
    }
    if (testfortoggle() )   {                   // Make sure both inputs are toggling.
     lcd.setCursor (0,1);
     lcd.print ("One Chan");                    // If only one, let user know
       // Run some basic tests to make sure we are detecting a real signal
     if ( ( time_between_pulses < 1 ) || ( pulsewidth < 1 ) )
     {
       lcd.setCursor (0,1);
       lcd.print ("No signal");                 // If not, print "no signal"
       Clearspace(3);   
     }  
    delay(300);
   }
   else {
    // If rotation is forward and the signals look real (pulsewidth and frequency are reasonable)
    if ( (rotationforward == true) && !( time_between_pulses < 1 ) || ( pulsewidth < 1 ) ) 
    {
      lcd.setCursor (0,1);
      lcd.print ("Forward ");                  // Tell the user rotation is in forward direction
    }
    // If rotation is in reverse and the signals look real (pulsewidth and frequency are reasonable)
    else if ( (rotationforward == false) && !( time_between_pulses < 1 ) || ( pulsewidth < 1 ) ) 
    {
      lcd.setCursor (0,1);
      lcd.print ("Reverse ");                  // Else tell the user rotation is in reverse direction
    } 
   }
    lcd.setCursor (8,1);
    if ((time_between_pulses+pulsewidth) > 1)  // If the speed measurement looks valid
    printjustify( 200000 / (time_between_pulses+pulsewidth) );  // display approx speed in CPS
    else  {
      lcd.setCursor (9,1);                 // Otherwise move over to leave room for no signal message
      Clearspace(2);                       // and print a few blanks
    }
    lcd.print (" CPS ");                   // Then label the output CPS for Counts Per Second
    time_between_pulses = 0;               // Zero the input values to insure we are not seeing stale values
    pulsewidth = 0;
  } 
  Clearscreen();                           // When done clear the screen
  ClearServos();                           // turn off motor controllers when this routine exits. And detach servo object 
  delay(Debounce);                         // Wait a short time to reduce the chance that you skip multiple states
  return;
} 


// Utility to test to see if DigitalIO2 is toggling.  Used with the encoder monitor.
boolean testfortoggle() {
  boolean sawhigh = false;
  boolean sawlow = false;
  for (long int testtime=0; testtime<60000; testtime++) { 
    if (digitalRead(DigitalIO2) == HIGH)          //  
        sawhigh = true;
    if (digitalRead(DigitalIO2) == LOW)          //  
        sawlow = true;
    if (sawhigh && sawlow)           //  
        return false;    
  }
  return true;
}


// This routine is a simple closed loop speed controller, but not real PID.
// It reads the potentiometer and calculates an intended speed.  But it also reads an encoder to find
// the current speed. It then makes a simple calculation of the PWM output based on bot the target
// speed and the current speed.  
void MySpeedControl(){ 
  ZeroKnob(PanelKnob);                     // make sure that the control knob is centered before starting output
  lcd.setCursor (0,0);
  lcd.print ("PWM with Feedback");         // Display what test mode we are in
  Clearline(1);    
  SetupServos();                           // Set up two servo outputs
  int ain;                                 // a variable to store the analog input value
  int currentspeed;
  int targetspeed;
  int pwmout;
  int pwmoutlast=90;
  time_between_pulses = 0;                 // Zero the input values to insure we are not seeing stale values
  pulsewidth = 0;  
  while (digitalRead(NextButton) == HIGH) {   // Run this routine until the "NEXT" key is hit
    if ( ( time_between_pulses < 1 ) || ( pulsewidth < 1 ) )
    {
      currentspeed = 0;
    }
    else if (rotationforward == true)       // If rotation is forward
    {
      currentspeed = (100000 / (time_between_pulses+pulsewidth) );
    }
    else if (rotationforward == false)       // If rotation is Reverse
    {
      currentspeed = -1 * (100000 / (time_between_pulses+pulsewidth) );
    }     
    ain = ReadPot(PanelKnob);                // reads the value of the potentiometer with a deadband
    lcd.setCursor (0,1);
    lcd.print ("TS   ");
    lcd.setCursor (3,1);
    lcd.print (ain/10);  
    Clearspace(3);  
    targetspeed = ain*3;    
    ain = 90+(targetspeed-currentspeed)/4;
    pwmout = (ain+pwmoutlast*10)/11;
    pwmoutlast = pwmout;   
    if (pwmout > 179) pwmout = 179;          // used for PWM output to account for potentiometers sometimes not reading full scale 
    if (pwmout < 0) pwmout = 0;   
    time_between_pulses = 0;                 // Zero the input values to insure we are not seeing stale values
    pulsewidth = 0; 
    lcd.setCursor (8,1);
    lcd.print ("PWM    ");
    lcd.setCursor (NextButton,1); 
    lcd.print (pwmout);   
    myservo1.write(pwmout);                      // Outputs the servo control 
    pwmout = map(pwmout, 0, 179, 179, 0);        // 
    myservo2.write(pwmout);                      // Outputs a copy of the servo control signal in reverse on a seperate I/0

  } 
  Clearscreen();                                 // When done clear the screen
  ClearServos();                                 // turn off motor controllers when this routine exits. And detach servo object 
  delay(Debounce);                               // Wait a short time to reduce the chance that you skip multiple states
  return;
} 



// This routine is designed to read digital pulses and display the frequency and pulse width.
void MyPulseCount(){ 
  ZeroKnob(PanelKnob); 
  int ain;                                 // a variable to store the analog input value
  lcd.setCursor (0,0);
  lcd.print ("Frequency Width");           // Display what test mode we are in
  Clearline(1);
  myservo1.attach(PWMOut1);                // attaches pin 3 to the first servo object 
  myservo1.write(90);                      // Make sure that PWM outputs start with motors turned off 
 
  while (digitalRead(NextButton) == HIGH) {        // Run this routine until the "NEXT" key is hit
    DualServoOut();                      // Outputs out two servo signals one the inverse of the other// Output a PWM signal to use to control a motor for convienience in testing
    if ( ( time_between_pulses < 1 ) || ( pulsewidth < 1 ) )
    {
      lcd.setCursor (0,1);
      lcd.print ("Not Detected");
      Clearspace(6);
    }
    else        // 
    {
      lcd.setCursor (0,1);
      printjustify(1000000/(time_between_pulses+pulsewidth) );  // And display approx speed
      Clearspace(1);
      lcd.print (" HZ   ");
      Clearspace(6);
      lcd.setCursor (10,1);
      ain = map(pulsewidth, 540, 2390, 1000, 2000);  
      printjustify( ain );
      lcd.setCursor (11,1);
      lcd.print (ain);
      lcd.setCursor (11,1);
     // lcd.print (".");
      lcd.setCursor (14,1);
      lcd.print ("ms");
    }   

    time_between_pulses = 0;               // Zero the input values to insure we are not seeing stale values
    rotationforward = true;                // Reset rotation direction. If it is reverse the interrupt service routine
                                           // will change it, but resetting it here helps to reduce size of the ISR
    pulsewidth = 0;
    delay(Debounce);                       // Control the screen refresh rate. 
  } 
  Clearscreen();                           // When done clear the screen
  myservo1.write(90);                      // turn off motor controllers when this routine exits. 
  myservo1.detach();                       // detach servo object 
 
  delay(Debounce);                         // Wait a short time to reduce the chance that you skip multiple states
  return;
} 




// This routine is designed to read and interpret signals intended to control a Spike H-Bridge relay.
void MySpikeInOut(){ 
  lcd.setCursor (0,0);
  lcd.print ("SpikeIn");                                      // Display what test output will be on this line
  pinMode(DigitalIO2, INPUT); 
  lcd.setCursor (0,1);
  lcd.print ("Output");                                       // Display what test output will be displayed here
  pinMode(SpikeOut1, OUTPUT); 
  pinMode(SpikeOut2, OUTPUT); 
  while (digitalRead(NextButton) == HIGH) {                   // Run this routine until the "NEXT" key is hit
    if (digitalRead(DigitalIO1) == digitalRead(DigitalIO2))   // If both input signals are the same
    {
      lcd.setCursor (9,0);
      Clearspace(2);
      lcd.print ("Stop");                                     // Tell the user this is the stop state
      Clearspace(6);
    }  
    else if (digitalRead(DigitalIO1) == HIGH)                 // If signal is for forward direction
    {
      lcd.setCursor (9,0);
      lcd.print ("Forward");                                  // Tell the user rotation is forward
      Clearspace(4);
    }
    else if (digitalRead(DigitalIO2) == HIGH)
    {
      lcd.setCursor (9,0);
      lcd.print ("Reverse");                                  // Else tell the user rotstion is reverse
        Clearspace(4);
    }  
    // Do a similar thing for the control buttons
    if (digitalRead(OptionButton1) == digitalRead(OptionButton2))   // If both input signals are the same
    {
      lcd.setCursor (9,1);
      lcd.print ("Stop");                                           // Tell the user this is the stop state
      Clearspace(5);
    }  
    else if (digitalRead(OptionButton1) == HIGH)                    // If signal is for forward direction
    {
      lcd.setCursor (9,1);
      lcd.print ("Forward");                                        // Tell the user rotation is forward
    }
    else if (digitalRead(OptionButton2) == HIGH)
    {
      lcd.setCursor (9,1);
      lcd.print ("Reverse");                                        // Else tell the user rotation is in reverse
    }  

    if (digitalRead(OptionButton1) == LOW)                          // If signal is for forward direction
    {
      digitalWrite(10, HIGH);
    }
    else
    {
      digitalWrite(10, LOW);
    }
    if (digitalRead(OptionButton2) == LOW)                          // If signal is for reverse direction
    {
      digitalWrite(11, HIGH);
    }
    else
    {
      digitalWrite(11, LOW);
    }
  } 
  Clearscreen();                                                    // When done clear the screen

    delay(Debounce);                // Wait a short time to reduce chance you skip multiple states
  return;
} 


// Read and display two analog input channels
void Myanalog(){ 
  int ain;                                   // a variable to store the analog input value
  while (digitalRead(NextButton) == HIGH) {  // Run this routine until the "NEXT" key is hit
    lcd.setCursor (0,0);
    lcd.print ("Analog ");
    ain = analogRead(AnalogIO1);             // reads the analog input (value between 0 and 1023)  
    ain = map(ain, 0, 1023, 0, 5000);        // scale input to read in voltage 0 to 5v
    lcd.setCursor (0,1);                     // Output the reading 
    printjustify(ain); 
    lcd.setCursor (1,1); 
    printjustify(ain); 
    Clearspace(2);
    lcd.setCursor (1,1);
    lcd.print (".");
    ain = analogRead(AnalogIO2);              // reads the analog input (value between 0 and 1023)
    ain = map(ain, 0, 1023, 0, 25000);        // scale input to read in voltage 0 to 25
    lcd.setCursor (10,1);                     // Output the reading
    printjustify (ain*5);                     // This input comes through a voltage divider, so multiply the value 
    if (ain > 10) ain = ain-10; {
      lcd.setCursor (11,1); 
      printjustify (ain*5);
    }
    Clearspace(2);
    lcd.setCursor (11,1);
    lcd.print (".");
    delay(100);  
  }
  Clearscreen();                              // When done clear the screen
  delay(Debounce);                            // Wait a short time to reduce the chance you skip multiple states
  return;
}

// Read and display temperature and acceleration from an ADW22307 sensor
// Also do some crude integration to get approximate angle
void Mygyro(){ 
  int position = 0;
  int zerorate;
  int rawrate;
  int rate = 0;
  zerorate = rawrate = Zerogyro();                     // read the gyro once to find the inertal state (hopefully)
  // Note: The accelerometer should not be moving when this function is started.
  // The inertal or non moving state will be compaired to future readings to determin if the device is moving
  while (digitalRead(NextButton) == HIGH) {  // Run this routine until the "NEXT" key is hit
    if (digitalRead(OptionButton1) == LOW)   // Use F1 to zero position
    position = 0;
    if (digitalRead(OptionButton2) == LOW)   // Use F2 to read a new inertal state
    zerorate = Zerogyro();
    lcd.setCursor (0,0);                     // Label the RATE display
    lcd.print ("Rate");
    
    rawrate = Readgyro();           // Subtract the inital value from all readings   
    rate = (rawrate - zerorate);     
//   if ((rate<=1)&&(rate>=-1)) rate=0;       // Add a deadband to reduce drift
 //   if (rate>1) rate=rate-1;       // Add a deadband to reduce drift
 //  if (rate<-1) rate=rate+1;       // Add a deadband to reduce drift

    
    lcd.setCursor (4,0);
    Clearspace(5);
    lcd.setCursor (4,0);
    printjustify (rate); 
    position = position + rate;              // Accumulate (integrate) rate values to get approx direction info
    lcd.setCursor (0,1);
    
    lcd.print ("Heading");
    lcd.setCursor (10,1);
    printjustify (map (position, -1000, 1000, -400, 400) );
    Clearspace(5); 
    lcd.setCursor (10,0);
    lcd.print ("T");                         // Also label a display for temperature
    printjustify (Readtemp());               // And print the temperature  
//    delay(1);                               // Add some amount of controled delay for better integration
  }
  Clearscreen();                             // When done clear the screen
  delay(Debounce);                           // Wait a short time to reduce the chance you skip multiple states
  return;
}

// Function to read a temperature sensor on analog port 2
// Scale factors are for the temperature sensor on a ADW22307 gyro
int Readtemp(){ 
  int ain;
  int temp;
  ain = analogRead(AnalogIO1);              // reads the analog input (value between 0 and 1023)   
  temp = map(ain, 0, 1023, -28, 527);     // scale input. Remember this input has a voltage divider
  return temp; 
}

// Function to read analog values from a ADW22307 gyro sensor
int Readgyro(){ 
  int cumlrate = 0;
  int averate;
  int count;
  int rate;
  for(count = 0; count < 6; count++ )  {   // Averages several readings to reduce noise
  rate = analogRead(AnalogIO2);               // reads the analog input (value between 0 and 1023)   
  cumlrate = (cumlrate + rate);             // part of the averaging function
  delay(1);
  }
  averate = cumlrate/6;                    // divide to get average acceleration
  return averate;                           // and return that value
}


// Function to read several analog values from a ADW22307 gyro sensor and average them
int Zerogyro(){ 
  int cumlrate = 0;
  int averate;
  int count;
  int rate;
  for(count = 0; count < 40; count++ )  {   // Averages several readings to reduce noise
  rate = Readgyro();                          // reads the analog input (value between 0 and 1023)   
  cumlrate = (cumlrate + rate);             // part of the averaging function
  delay(10);
  }
  averate = cumlrate/40;                    // divide to get average acceleration
  return averate;                           // and return that value
}

// Test a switch or 5 volt digital input
// The test is done using an analog input to all testing for weak inputs
void Myswitch(){ 
  int ain;                                   // a variable to store the analog input value
  digitalWrite(A1, HIGH);                    // set pullup on analog pin 0
  while (digitalRead(NextButton) == HIGH) {  // Run this routine until the "NEXT" key is hit
    lcd.setCursor (0,0);
    lcd.print ("Switch Tester");
    ain = analogRead(AnalogIO1);             // reads the analog input (value between 0 and 1023)     
    lcd.setCursor (0,1);                     // Output the reading
    lcd.print ("weak chk connect"); 
    if (ain <= 82)                           // If we have a solid digital low signal
      {
        lcd.setCursor (0,1);                 // Output the reading
        lcd.print ("Switch is on"); 
        Clearspace(4);
      }    
       if (ain >= 900)                       // If we have a solid digital high signal
      {
        lcd.setCursor (0,1);                 // Output the reading
        lcd.print ("Switch is off"); 
        Clearspace(4);
      }     

    delay(Debounce);  
  }
  Clearscreen();                            // When done clear the screen
    digitalWrite(A1, LOW);                  // turn off the pullup on analog pin 0
    delay(Debounce);                        // Wait a short time to reduce the chance you skip multiple states
  return;
}

// A function that outputs a relay control signal based on digital input.
// With adequate care this might be used to control a compressor from a pressure switch
void Mycompress(){ 
  int ain;                                   // a variable to store the analog input value
  digitalWrite(A1, HIGH);                    // set pullup on analog pin 1
  digitalWrite(SpikeOut1, LOW);              // Make sure compressor is off
  digitalWrite(SpikeOut2, LOW);              // Never run the compressor in reverse
  while (digitalRead(NextButton) == HIGH) {  // Run this routine until the "NEXT" key is hit
    lcd.setCursor (0,0);
    lcd.print ("Compressor Contl");
    ain = analogRead(AnalogIO1);             // reads the analog input (value between 0 and 1023)  
          
   
    if (ain <= 82)                           // If we have a solid digital low signal
      {
        lcd.setCursor (0,1);                 // Turn on the compressor
        lcd.print ("Comp is on"); 
        Clearspace(2);
        digitalWrite(SpikeOut1, HIGH);
      }    
       if (ain >= 83)                        // If we have digital high signal
      {
        lcd.setCursor (0,1);                 // Output the reading
        lcd.print ("Comp is off"); 
        Clearspace(5);
        digitalWrite(SpikeOut1, LOW);
      }     

    delay(100);  
  }
  Clearscreen();                            // When done clear the screen
    digitalWrite(10, LOW);                  // Make sure compressor is off
    digitalWrite(A1, LOW);                  // turn off the pullup on analog pin 0
    delay(Debounce);                        // Wait a short time to reduce the chance you skip multiple states
  return;
}


// This function is an I2C address scanner and is
// based on example code from the Arduino.cc forum.  
// It has been adapted to work with the LCD display on the supertester.
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.   
void I2CScan()
{
  Wire.begin(); 
  lcd.setCursor (0,0);
  lcd.print("I2C Start Scan");
  Clearline(1); 
  delay(300); 
  while (digitalRead(NextButton) == HIGH) {            // Run this routine until the "NEXT" key is hit
    byte error, address;
    int nDevices;

    lcd.setCursor (0,0);
    lcd.print("Scanning");
    lcd.setCursor (0,1);
    delay(200);
    nDevices = 0;
    for(address = 1; address < 127; address++ ) 
    {
      // The i2c_scanner uses the return value of the Write.endTransmisstion to see if
      // a device did acknowledge at that address.
      if (digitalRead(NextButton) == LOW)   // Allow an early exit.
        return;
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
      if (error == 0)
      {
        lcd.setCursor (0,1);
        lcd.print("I2C dev at 0x");
        if (address<16) 
          lcd.setCursor (0,1);
        lcd.print("0");
        lcd.print(address,HEX);
        Clearspace(2);
        devaddr = address;
        delay(400);
        if (digitalRead(NextButton) == LOW)
          return;
        delay(200);
        nDevices++;
      }
      else if (error==4) 
      {
        lcd.setCursor (0,1);
        lcd.print("Err at 0x");
        if (address<16) 
          lcd.print("0");
        lcd.print(address,HEX);
        Clearspace(5);
        delay(400);
      }  
    }
    if (nDevices == 0)
    {
      lcd.setCursor (0,1);
      lcd.print("No devices found");
    }
    else
    {
      lcd.setCursor (0,0);
      lcd.print("done");
      Clearspace(11);
    }
    delay(400);           // wait .4 seconds for next scan  
  }
  Clearscreen();                          // When done clear the screen
  delay(Debounce);                             // Wait a short time to reduce the chance you skip multiple states
  return;
}

// Code for reading some values from the accelerometer.
void Accell()
{
  Wire.begin();        // join i2c bus (address optional for master) 
  //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  writeTo(DATA_FORMAT, 0x01);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeTo(POWER_CTL, 0x08);
  while (digitalRead(NextButton) == HIGH)     // Run this routine until the "NEXT" key is hit
 { 
    readAccel();                              // read the x/y/z vales from the accelerometer
    delay(Debounce);                          // only read every 0.5 seconds
  }
  Clearscreen();                              // When done clear the screen
  delay(100);                                 // Wait a short time to reduce the chance you skip multiple states
  return;
}

//  An\ crude example of how values from the accelerometer can be used to determine orentation
void AccellFun()                              // A fun variation on reading accelerometer data.
{
  Wire.begin();                               // join i2c bus (address optional for master) 
  //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  writeTo(DATA_FORMAT, 0x01);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeTo(POWER_CTL, 0x08);
  while (digitalRead(NextButton) == HIGH)     // Run this routine until the "NEXT" key is hit
  { 
    readAccelfun(); // read the x/y/z vales from the accelerometer
    delay(500); // only read every 0.5 seconds
  }
  Clearscreen();                              // When done clear the screen
  delay(Debounce);                            // Wait a short time to reduce the chance you skip multiple states
  return;
}



/************************************************************************
 *  The accelerometer portion of this code is based on the               *
 *  Bare bones ADXL345 I2C example for Arduino 1.0                       *
 *  by Jens C Brynildsen <http://www.flashgamer.com>                     *
 *  but modified for use on the supertester hardware                     *
 ***********************************************************************/
// reads acceleration values for each axis and writes them to the display 
void readAccel() {
  uint8_t howManyBytesToRead = 6;
  readFrom( DATAX0, howManyBytesToRead, _buff); //read the acceleration data from the ADXL345
  // each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
  // thus we are converting both bytes in to one int
  int x = (((int)_buff[1]) << 8) | _buff[0];   
  int y = (((int)_buff[3]) << 8) | _buff[2];
  int z = (((int)_buff[5]) << 8) | _buff[4];

  if ((x == 0)||(y==0)||(z==0))
  {
    // This state is very unlikely to ever occur naturally except in orbit
    // or a long freefall.  So if it does, it is most likely that there is   
    // no accelerometer at this address.  
    lcd.setCursor (0,0);
    lcd.print("Device not found");
    lcd.setCursor (0,1);
    lcd.print("Try scaning I2C"); 
  }  
  else
  {
    Clearscreen();

    lcd.setCursor (0,0);
    lcd.print("ADXL345 X: ");
    lcd.print(x);

    lcd.setCursor (0,1);
    lcd.print("Y: ");
    lcd.print(y);

    lcd.setCursor (6,1);
    lcd.print("  Z: ");
    lcd.print(z);
  }
  // Wait a short time to allow the Screen to be read
  delay(100); 

}

// This function is part of the I2C capability  See source credit elsewhere in main code
// Writes bytes to address register on I2C device 
void writeTo(byte address, byte val) {
  Wire.beginTransmission(devaddr);  // start transmission to device 
  Wire.write(address);              // send register address
  Wire.write(val);                  // send value to write
  Wire.endTransmission();           // end transmission
}

// This function is part of the I2C capability  See source credit elsewherein main code
// Reads num bytes starting from address register on I2C device in to _buff array
void readFrom(byte address, int num, byte _buff[]) {
  Wire.beginTransmission(devaddr);  // start transmission to device 
  Wire.write(address);              // sends address to read from
  Wire.endTransmission();           // end transmission
  Wire.beginTransmission(devaddr);  // start transmission to device
  Wire.requestFrom(devaddr, num);     // request 6 bytes from device
  int i = 0;
  while(Wire.available())            // device may send less than requested (abnormal)
  { 
    _buff[i] = Wire.read();          // receive a byte
    i++;
  }
  Wire.endTransmission();            // end transmission
}


// This is an extra function demonstrates how accelerometer data might be used.
// The simple example reads data from an ADXL345 accelerometer.  But this time instead
// of displaying raw data it converts the data into some approximate device orientations.
void readAccelfun() {
  uint8_t howManyBytesToRead = 6;
  readFrom( DATAX0, howManyBytesToRead, _buff); //read the acceleration data from the ADXL345
  // each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
  // thus we are converting both bytes in to one int
  int x = (((int)_buff[1]) << 8) | _buff[0];   
  int y = (((int)_buff[3]) << 8) | _buff[2];
  int z = (((int)_buff[5]) << 8) | _buff[4];
  lcd.setCursor (0,0);
  lcd.print("I am sitting");
  Clearline(1);
  lcd.setCursor (0,1);
  if (x>100) 
    lcd.print("on my left side"); 
  if (x<-100) 
    lcd.print("on my right side");
  if (y>100) 
    lcd.print("on my end"); 
  if (y<-100) 
    lcd.print("on my other end");
  if (z>100) 
    lcd.print("upright");
  if (z<-100) 
    lcd.print("upside down");
    Clearspace(8);
   delay(100);              // Wait a short time to allow the Screen to be read
}


// This function implimemts a simple stop watch with Hours, Minutes, Seconds, and Miliseconds.
// However, unlike a normal stop watch this one can be triggered by limit switches of other external IO.
void StopWatch(){ 
  int Minutes = 0;                            // Some variables to store time data
  int Sec = 0;
  int Msec = 0;
  int Hours = 0;
  lcd.setCursor (0,0);                        // set the cursor to column 0, line 0
  lcd.print("F1=Start F2=Stop");              // Print a screen message to the LCD. 
  lcd.setCursor (0,1);                        // Position for the second line where timing will be displayed
  while (digitalRead(NextButton) == HIGH) {   // Run this routine until the "NEXT" key is hit
    if (analogRead(SelectButton) < 500)   {   // Use the select button to re zero all the counters
      Minutes = Sec = Msec = Hours = 0;
      Clearline(1);
    }
    if ((digitalRead(OptionButton1) == LOW) || (digitalRead(DigitalIO1) == HIGH) )    {   // Wait for start signal
      unsigned long start = millis();  // Variable to keep track of the starting time
      unsigned long now;               // Variable to keep track of the current time so we can calculate elapsed time
      while((digitalRead(OptionButton2) == HIGH) && (digitalRead(DigitalIO2) == LOW) ) {  // Count until stop signal
        now = millis();                // Check current time
        Msec = (now-start);            // Calculate elapsed time
        if (Msec >= 1000)  {            // If the elapsed miliseconds equals 1 second
          Msec = 0;                      // Roll miliseconds over
          Sec = Sec + 1;                 // Increment seconds
          now = millis();              // And reset counters
          start = millis();
        }
        if (Sec >= 60)  {                // If elapsed seconds equal 1 minute
          Sec = 0;                       // Roll seconds over into minutes
          Minutes = Minutes + 1;
        }
        if (Minutes >= 60)  {            // If elapsed seconds equal 1 hour
          Minutes = 0;                   // Roll minutes over into hours
          Hours = Hours + 1;
        }
        lcd.setCursor (11,1);            // Display hours, minutes, seconds, and milliseconds.
        printjustify (Msec);        
        lcd.setCursor (7,1); 
        printjustify (Sec);
        lcd.setCursor (3,1);      
        printjustify (Minutes);
        lcd.setCursor (0,1); 
        printjustify (Hours);
      }  
    }
  } 
  Clearscreen();                       // When done clear the screen
  delay(Debounce);                     // Wait a short time to reduce the chance you skip multiple states
  return;
} 

// This function produces sound to simulate a sonic screwdriver.
// It requires a speaker to be connected to digital I/O 1
// Based on the melody example in the Arduino examples
void Sonic(){ 
  Clearscreen();                       // clear the screen
  lcd.setCursor (0,0);                 // the cursor to column 0, line 0
  lcd.print("Connect Speaker");        // Print a Flash Screen message to the LCD.  (May be customized as desired)
  lcd.setCursor (0,1);                 // set the cursor to column 2, line 0
  lcd.print("and press F1");              // Print a Flash Screen message to the LCD.  (May be customized as desired)
 int melody[] = { NOTE_F7, NOTE_G4, NOTE_A2};
 int noteDurations[] = {  9,12,7 };
 int x = 0;
  while (digitalRead(NextButton) == HIGH) {        // Run this routine until the "NEXT" key is hit
   for (int thisNote = 0; thisNote < 3; thisNote++) {
    int noteDuration = 1000/noteDurations[thisNote];    
    if (digitalRead(OptionButton1) == LOW)
     tone(DigitalIO1, melody[thisNote],noteDuration);  
    int pauseBetweenNotes = noteDuration * .14;
    delay(pauseBetweenNotes);
    noTone(DigitalIO1);
  } 
 } 
Clearscreen();           // When done clear the screen
delay(Debounce);         // Wait a short time to reduce the chance you skip multiple states
return;
} 





// This function just displays a default message for menu functions that are not yet implimented
void Dummycode(){ 
  lcd.setCursor (0,0);                 // set the cursor to column 0, line 0
  lcd.print("Your code");              // Print a Flash Screen message to the LCD.  (May be customized as desired)
  lcd.setCursor (0,1);                 // Position for the second line of text
  lcd.print("could be here!");         // Print line 2 of the flash screen  (May be customized as desired)
  while (digitalRead(NextButton) == HIGH) {        // Run this routine until the "NEXT" key is hit
  } 
  Clearscreen();                           // When done clear the screen
    delay(Debounce);                            // Wait a short time to reduce the chance you skip multiple states
  return;
} 



// This function just displays information about our team
// If you have reused this code with only minor changes please leave the link to our website
// and change the message to read something like "Based on" "Supertester" 
void About(){ 
  lcd.setCursor (0,0);                 // set the cursor to column 0, line 0
  lcd.print("Proud product of");       // Print a Flash Screen message to the LCD.  (May be customized as desired)
  lcd.setCursor (0,1);                 // Position for the second line of text
  lcd.print("Spartan Robotics");       // Print line 2 of the flash screen  (May be customized as desired)
  delay(2000); 
  lcd.setCursor (0,0);
  lcd.print ("FRC TEAM 997");      // Display what test mode we are in
  Clearspace(5);
  lcd.setCursor (0,1);
  lcd.print ("chsrobotics.org ");
  while (digitalRead(NextButton) == HIGH) {    // Run this routine until the "NEXT" key is hit
    // Hidden feature (easter egg)
    // After viewing the about information a user can press both function buttons to reveal a
    // special feature of the tester.
    if ((digitalRead(OptionButton1) == 0) && (digitalRead(OptionButton2) == 0))   // If both input signals are the same
    {
      myGame();
      delay(Debounce);                        // Wait a short time to reduce the chance you skip multiple states
      Clearline(0);     
      lcd.setCursor (3,0);
      lcd.print("Play again?");       // Print a 
    }
  } 
  Clearscreen();                       // When done clear the screen

    delay(Debounce);                        // Wait a short time to reduce the chance you skip multiple states
  return;
} 

// This portion of the code adapted from an example by @TheRealDod
/* Simple Car game for a 16x2 LCD display 
   Enjoy,
   @TheRealDod, Nov 25, 2010
*/
void myGame() 
{
  crash = crashtime = road_index = 0;
  step_duration = MAXSTEPDURATION;
  line_buff[1+ROADLEN] = '\0'; // null terminate it
  randomSeed(analogRead(PanelKnob));
  for (int i=0; i<NGLYPHS; i++) {
    lcd.createChar(i+1,glyphs[i]);
  }
  for (int i=0; i<ROADLEN; i++) {
    road[i]=-1;
  }
  lcd.begin(16,2);
  getSteeringWheel();
  drawRoad();
  lcd.setCursor(1,0);
  lcd.print(INTRO1);
  lcd.setCursor(1,1);
  lcd.print(INTRO2);
  delay(INTRODELAY);
   unsigned long start = millis();
 
while(digitalRead(NextButton) == HIGH) {          // Run this routine until the "NEXT" key is hit) 
  unsigned long now = millis()-INTRODELAY;
  if (!crash) {
    getSteeringWheel();
    crash = (car_pos==road[road_index]);
  }
  if (crash) {
    if (!crashtime) {
      crashtime=now;
      drawRoad();
      // Game over text
      // (keep first 2 "crash" columns intact)
      lcd.setCursor(2,0);
      lcd.print("Crashed after");
      lcd.setCursor(2,1);
      lcd.print((now-start)/1000);
      lcd.print(" seconds.");
    }
    delay(10); // Wait a bit between writes
  } 
  else {
 
    int prev_pos = road[(road_index-1)%ROADLEN];
    int this_pos = random(MAXROADPOS);
    while (abs(this_pos-prev_pos)<2) { // don't jam the road
      this_pos = random(MAXROADPOS);
    }
    road[road_index] = this_pos;
    road_index = (road_index+1)%ROADLEN;
    drawRoad();
    delay(step_duration);
    if (step_duration>MINSTEPDURATION) {
      step_duration--; // go faster
    }
  }
}
}

void getSteeringWheel() {
  car_pos = map(analogRead(PanelKnob),0,1024,0,NCARPOSITIONS);
}
 
void drawRoad() {
  for (int i=0; i<2; i++) {
    if (crash) {
      line_buff[0]=crash2glyphs[car_pos][i];
    } 
    else {
      line_buff[0]=car2glyphs[car_pos][i];
    }
    for (int j=0; j<ROADLEN; j++) {
      int pos = road[(j+road_index)%ROADLEN];
      line_buff[j+1] = pos>=0 && pos<NCARPOSITIONS ? truck2glyphs[pos][i] : BLANK;
    }
    lcd.setCursor(0,i);
    lcd.print(line_buff);
  }
}
/*
Ideas to add
strobe or tach
thermometer
gyro including integration and temperature correction
other encoder formats
Read Sonar

*/

