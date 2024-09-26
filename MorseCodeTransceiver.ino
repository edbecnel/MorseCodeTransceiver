/*
 *             RS Version
 *        Morse Code Transceiver
 *        Using a Beetle CM-32U4
 *              04/20/24
 */
/*           Notes
 *04/20/24 Adapted the code to the 2 Beetle pinouts and eliminated the photoresistor and LED
 *         Wired direct to each beetle pins
 *         Changed from a 16,2 LED Display to a 20,4 LED Display
 *04/23/24 Added Keyboard
 *04/28/24 added a 2nd LCD to receve the sent text
 *04/30/24 ED B added a keyboard buffer to TX the text that is typed
 *         into the LCD 1 display when enter key is pressed
 *08/28/24 Testing the input CW Sign to Square Wave LM567 Tone Decoder to.
           The current code below as a sender does not convert the dots and dashs to
           a 400-1200Hz tone to send CW over a radio needs this adding.
           Needs to turn each dot/dash into a CW tone to output to Radio.
 *08/29/24 Added the switch code for 400-1200Hz tone settings to match the incomming CW tone, and then
           send the matching CW tone over the radio.
           Added the serial input section to test the switch code tone settings
 *09/17/24 EdB: Integrating morseduino_i2c-1.ino with this source
 *09/26/24 EdB: Code cleanup.
 */

//        Include the Libraries:

#include <Wire.h>
#include <LiquidCrystal_I2C.h> // for LCD 1
#include <LiquidCrystal_I2C.h> // for LCD 2
#include <PS2KeyAdvanced.h>
#include "AD5245.h" // Digi-Pot

#include "I2C_eeprom.h"

// the address of your EEPROM
#define DEVICEADDRESS (0x50)

// this must start on a page boundary!
#define START_PAGE_ADDR 0
#define SHORT_BUFFER_LEN 4

// used for multi-page writes
#define LONG_BUFFER_LEN 64

// Use this for 24LC04
I2C_eeprom eeprom(0x50, I2C_DEVICESIZE_24LC04);
// EdB

// EdB: Used to test time used to determine the EEPROM size
uint32_t start, diff = 0;

unsigned int addr = START_PAGE_ADDR;
#define LCDCOLS 20 // Our LCD is 20x4  RS. May or may not need based on the other LCD code in the other file
#define LCDROWS 4

// #undef PS2_REQUIRES_PROGMEM
#define PS2_REQUIRES_PROGMEM

#define HISCALL "WB7FHC"   // Callsign of kit builder  RS. Change this to me
#define VDATE "2016.03.07" // Sketch Version Date  RS. Change date
// #define DEBUG 1                     // remove '//' to turn monitor debugging on

// *************************************
// ******** Begin Morseduino definitions
// *************************************
// EdB Note: We will be using the LiquidCrystal_I2C library and related code here instead of morseduino_i2c's LCD usage

// RS. need the below to end mark
#define VERSIONDELAY 1500 // Pause before erasing version date

#define MAXSWEEP 240      // tone decoder chip cycles around if we go too high with the digital pot
#define SWEEPCOUNTBUMP 10 // how much more we will delay checking for tone match after each sweep

#define SCOPEPIN 5      // Scope Channel 2 D5 RS. will neeed changing.
#define RIGHT_BUTTON 6  // Switch 2 RS. will neeed changing.
#define LEFT_BUTTON 7   // Switch 1 RS. will neeed changing.
#define SIGNALPIN 8     // Key UP/DOWN is read here from both tone decoder chip and telegraph key RS. will neeed changing.
                        // Hook the key up between D8 and GND
                        // Also Scope Channel 1 D8
                        // hook osciliscope up to pins D5 and D8 to see tone decoder at work
int signalPinState = 1; // will store the value we read on this pin RS. will neeed changing.

#define SPEAKERPIN 9 // Used to drive a simple speaker if you want to hear your own code RS. will neeed changing.

// EdB - Start
// #define SLAVESELECTPIN 10 // connected to Pin 1 CS (Chip Select) of digital potentiometer 41010
// EdB - End

int myMax = 0;   // used to capture highest value that decodes tone
int myMin = 255; // used to capture lowest value that decodes tone

// EdB: Function Declarations
void intToByte(int intVal, byte byteSet[]);
int byteToInt(byte byteSet[]);
void eepromWriteInt(unsigned int pageAddress, int intVal);
int eepromReadInt(unsigned int pageAddress);
void setSideToneFromIndex();

int ToneSet = eepromReadInt(addr); // a value bumped up or down by the left button
int oldToneSet = ToneSet;        // so we can tell when the value has changed
int pitchDir = 1;                // Determines whether left button increases or decreases side-tone pitch
// RS. End of what is needed here

// RS. need the below to end mark
// noiseFilter is a value used to filter out noise spikes comming from the decoder chip
// the value of noiseFilter will be the number of milliseconds we wait after reading pin 8
// before reading it again to see if we have a valid key down or key up.

int noiseFilter = eepromReadInt(addr + 1);

int oldNoiseFilter = noiseFilter; // used to see if the value has changed

int sweepCount = 0; // used to slow down the sweep each time it cycles without success

// sideToneIndex is a value stored in EEPROM that determines the frequency of the sidetone
// values from 1 to 8 are allowed. The frequency of the tone will be this index times 110 Hz

int sideToneIndex = eepromReadInt(addr + 2);
int sideTone = 440; // the frequency of the side tone. Initialized to 440 but will change to match sideToneIndex in setupMorseduino()

/////////////////////////////////////////////////////////////////////////////////////////
// The following variables store values collected and calculated from millis()
// For various timing functions

long pitchTimer = 0; // keep track of how long since right button was pressed so we can reverse direction of pitch change
long downTime = 0;   // How long the tone was on in milliseconds
long upTime = 0;     // How long the tone was off in milliseconds

long startDownTime = 0; // Arduino's internal timer when tone first comes on
long startUpTime = 0;   // Arduino's internal timer when tone first goes off
long lastChange = 0;    // Keep track of when we make changes so that we can
                        // postpone writing changes to EEPROM until all have been made

long lastDahTime = 0; // Length of last dah in milliseconds
long lastDitTime = 0; // Length of last dit in milliseconds

// The following values will auto adjust to the sender's speed
long averageDah = 100; // A dah should be 3 times as long as a dit
long fullWait = 6000;  // The time between letters
long waitWait = 6000;  // The time between dits and dahs
long newWord = 0;      // The time between words
long dit = 10;         // We start by defining a dit as 10 milliseconds
// RS. End of what is needed here

// RS.  the Below LCD code may not be needed
// EdB: Review every place where the following variables are referenced and determine what needs
//      to change to remove the need for them.
////////////////////////////////////////////////////////////////////////////////////////////
// These variables handle line scrolling and word wrap on the LCD panel                   //
int LCDline = 1;           // keeps track of which line we're printing on                //
int lineEnd = LCDCOLS + 1; // One more than number of characters across display          //
int letterCount = 0;       // keeps track of how may characters were printed on the line //
int lastWordCount = 0;     // keeps track of how may characters are in the current word  //
int lastSpace = 0;         // keeps track of the location of the last 'space'            //
//                                                                                        //
// The next line stores the text that we are currently printing on a line,                //
// The charcters in the current word,                                                     //
// Our top line of text,                                                                  //
// Our second line of text,                                                               //
// and our third line of text                                                             //
// For a 20x4 display these are all 20 characters long                                    //
char currentLine[] = "12345678901234567890"; //
char lastWord[] = "                    ";    //
char line1[] = "                    ";       //
char line2[] = "                    ";       //
char line3[] = "                    ";       //
////////////////////////////////////////////////////////////////////////////////////////////

// RS. need the below to end mark
boolean ditOrDah = true;      // We have either a full dit or a full dah
boolean characterDone = true; // A full character has been received
boolean justDid = true;       // Makes sure we only print one space during long gaps
boolean speaker = false;      // We need to know if a speaker is connected
boolean sideToneSet = false;  // We need to know if we are changing the side tone

int myBounce = 2; // Used as a short delay between key up and down
int myCount = 0;

int wpm = 0; // We'll print the sender's speed on the LCD when we scroll

int myNum = 0; // We will turn dits and dahs into a binary number stored here

/////////////////////////////////////////////////////////////////////////////////
// Now here is the 'Secret Sauce'
// The Morse Code is embedded into the binary version of the numbers from 2 - 63
// The place a letter appears here matches myNum that we parsed out of the code
// #'s are miscopied characters
char mySet[] = "##TEMNAIOGKDWRUS##QZYCXBJP#L+FVH09#8###7#####/-61#######2###3#45";
char lcdGuy = ' '; // We will store the actual character decoded here

//////////////////////////////////////////////////////////
// define two special characters for LCD a didit and a dah
byte didit[8] = {
    B00000,
    B00000,
    B11011,
    B11011,
    B11011,
    B00000,
    B00000,
};

byte dahdah[8] = {
    B00000,
    B00000,
    B11111,
    B11111,
    B11111,
    B00000,
    B00000,
};
// RS. End of what is needed here

// *************************************
// ******** End Morseduino definitions
// *************************************

int ComCommand = 0; // Serial input to adjust tones as needed

//,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
int audio = 9; // Output Square Wave audio on pin 9. Use this audio to modulate the Morse dots and dashes
// #define note 400
int note = 400; // changed #define note to an int varable
// int note = 1000;
//,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
int val1 = 65; // Digi-Pot 1 startup Setting 0 - 256 prob needs changing
int val2 = 65; // Digi-Pot 2 startup Setting 0 - 256

AD5245 AD1(0x2C); //  AD1 == GND Digi-Pot 1. EdB - Used on the Tone Decoder input side?
AD5245 AD2(0x2D); //  AD2 == Vcc Digi-Pot 2. EdB - Used on the Audio Amplifier output side?

int timeUnitLength = 10;      // Speed of the morse time unit, 5 did not work, 10 seams to work ok for fastest
                              // speed. A slower speed will prob work better
int MorseLed = 9;             // Morse sender LED, on Beetle pin 9 - D9
                              // ED..!!  this needs to be mixed with the audio/note setting so that
                              // each dot/dash is a CW tone
int ControlLed = LED_BUILTIN; // Feedback LED
int photoResistorPin = 10;    // Photoresistor pin recever (A1 is default), on Beetle pin 4 A1
                              // this input needs to be D10 now need to change this Pin Name to reflect new changes
int lcdAdress1 = 0x27;        // LCD 1 I2C Address (labeled on the back of the screen)
int lcdAdress2 = 0x26;        // LCD 2 I2C Address (labeled on the back of the screen)

LiquidCrystal_I2C lcd(lcdAdress1, 20, 4);  // lCD1 displays typed messages to be transmitted
LiquidCrystal_I2C lcd2(lcdAdress2, 20, 4); // LCD2 displays messages received

//--------------------
// USER SETTINGS
//--------------------

/* Keyboard constants  Change pins to suit your Arduino
   define pins used for data and clock from keyboard */
#define DATAPIN 1 // Beetle uses Pins 0, 1
#define IRQPIN 0

/* LCD Constants to match your display */
/* Columns and Rows in display */
#define MAX_COL 20 // was 16
#define MAX_ROW 4  // was 2

// current cursor position
int8_t cols = 0;
int8_t rows = 0;

//--------------------

//--------------------
// DEBUG SETTINGS
//--------------------
bool debug = false;            // General debug
bool debugSensor = false;      // debug the light value threshold
bool debugtimeUnitLen = false; // timeUnitLen debug -> Only displays when touching the potentiometer -> can leave always on
bool debugTiming = false;      // debug the morse unit rules (Some clients don't follow the normal rules and their spaces and end of word are too short!)
bool debugAdv = false;         // Advanced debug: prints all the decision values
bool debugSender = false;      // Shows the different steps of the morse sending
bool debugPlotter = false;     // Plotter to facilitate the understanding of thresholds. To see it, open the serial plotter in your arduino IDE.
//--------------------

// Morse tree used for decoding
#ifdef PS2_REQUIRES_PROGMEM
const char MorseTree[] PROGMEM = {'\0', 'E', 'T', 'I', 'A', 'N', 'M', 'S',
                                  'U', 'R', 'W', 'D', 'K', 'G', 'O', 'H',
                                  'V', 'F', 'U', 'L', 'A', 'P', 'J', 'B',
                                  'X', 'C', 'Y', 'Z', 'Q', '\0', '\0', '5',
                                  '4', '\0', '3', '\0', '\0', '\0', '2', '\0',
                                  '\0', '+', '\0', '\0', '\0', '\0', '1', '6',
                                  '=', '/', '\0', '\0', '\0', '(', '\0', '7',
                                  '\0', '\0', '\0', '8', '\0', '9', '0', '\0',
                                  '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
                                  '\0', '\0', '\0', '?', '_', '\0', '\0', '\0',
                                  '\0', '"', '\0', '\0', '.', '\0', '\0', '\0',
                                  '\0', '@', '\0', '\0', '\0', '\0', '\0', '\0',
                                  '-', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
                                  '\0', ';', '!', '\0', ')', '\0', '\0', '\0',
                                  '\0', '\0', ',', '\0', '\0', '\0', '\0', ':',
                                  '\0', '\0', '\0', '\0', '\0', '\0', '\0'};
#else
const char MorseTree[] = {'\0', 'E', 'T', 'I', 'A', 'N', 'M', 'S',
                          'U', 'R', 'W', 'D', 'K', 'G', 'O', 'H',
                          'V', 'F', 'U', 'L', 'A', 'P', 'J', 'B',
                          'X', 'C', 'Y', 'Z', 'Q', '\0', '\0', '5',
                          '4', '\0', '3', '\0', '\0', '\0', '2', '\0',
                          '\0', '+', '\0', '\0', '\0', '\0', '1', '6',
                          '=', '/', '\0', '\0', '\0', '(', '\0', '7',
                          '\0', '\0', '\0', '8', '\0', '9', '0', '\0',
                          '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
                          '\0', '\0', '\0', '?', '_', '\0', '\0', '\0',
                          '\0', '"', '\0', '\0', '.', '\0', '\0', '\0',
                          '\0', '@', '\0', '\0', '\0', '\0', '\0', '\0',
                          '-', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
                          '\0', ';', '!', '\0', ')', '\0', '\0', '\0',
                          '\0', '\0', ',', '\0', '\0', '\0', '\0', ':',
                          '\0', '\0', '\0', '\0', '\0', '\0', '\0'};
#endif // PS2_REQUIRES_PROGMEM

/***** from original receiving part *********/
int val = 0; // Light value fron the LDR  (this is prob not needed any more)
int lightHigh = 0;
int lightLow = 0;
int codePtr = 0;
int previousLen = 200;
bool notAnalysed = false; // Keep tracks of wether or not we have treated the previous "State"
bool endOfTrans = true;   // Are we still transmitting? Important to print the last letter of the transmission
static unsigned long timer = millis();
bool lightState = false; // state of the light
int lightOnLen = 0;
int lightOffLen = 0;
int threshold = 0; // Will hold the threshold of the photoresistor to differanciate a dash from a dot.
                   // ED..!!  a lot of the photo sections could go away
int lcdPos = 0;    // Help track the lcd position to switch lines
int lcd2Pos = 0;   // Help track the lcd2 position to switch lines

/***** from original sender part *********/
// unsigned long signal_len,t1,t2; // For Manual morse code operations
String morseCode = ""; // Containes the morse code to be sent
// byte morseCodeLength = 0;
char i;               // BAZINGA can be removed right? (if translate gets the input directly)
String code = "";     // BAZINGA bundle with morseCode?
String message = "";  // Containes the message that needs to be sent
String txBuffer = ""; // Contains the actual state of the LED to transmit (H for high / L for Low and S as a start trigger)

// EdB: Start code section - Added support for sending morse code from keyboard buffer
String keybrdBuffer = ""; // Contains the message read from the keyboard to be sent as morse code when <Enter> ("Send") key is pressed.
bool sendPressed = false;
// EdB: End code section

unsigned long ledChangeTimer; // time of the next state change

// messages constants
/* Key codes and strings for keys producing a string */
/* three arrays in same order ( keycode, string to display, length of string ) */
#ifdef PS2_REQUIRES_PROGMEM
const uint8_t codes[] PROGMEM = {PS2_KEY_SPACE, PS2_KEY_TAB, PS2_KEY_ESC, PS2_KEY_DELETE,
                                 PS2_KEY_F1, PS2_KEY_F2, PS2_KEY_F3, PS2_KEY_F4,
                                 PS2_KEY_F5, PS2_KEY_F6, PS2_KEY_F7, PS2_KEY_F8,
                                 PS2_KEY_F9, PS2_KEY_F10, PS2_KEY_F11, PS2_KEY_F12};
const char spacestr[] PROGMEM = " ";
const char tabstr[] PROGMEM = "[Tab]";
const char escstr[] PROGMEM = "[ESC]";
const char delstr[] PROGMEM = "[Del]";
const char f1str[] PROGMEM = "[F1]";
const char f2str[] PROGMEM = "[F2]";
const char f3str[] PROGMEM = "[F3]";
const char f4str[] PROGMEM = "[F4]";
const char f5str[] PROGMEM = "[F5]";
const char f6str[] PROGMEM = "[F6]";
const char f7str[] PROGMEM = "[F7]";
const char f8str[] PROGMEM = "[F8]";
const char f9str[] PROGMEM = "[F9]";
const char f10str[] PROGMEM = "[F10]";
const char f11str[] PROGMEM = "[F11]";
const char f12str[] PROGMEM = "[F12]";

// Due to AVR Harvard architecture array of string pointers to actual strings
const char *const keys[] PROGMEM = {
    spacestr, tabstr, escstr, delstr, f1str, f2str,
    f3str, f4str, f5str, f6str, f7str, f8str,
    f9str, f10str, f11str, f12str};
const int8_t sizes[] PROGMEM = {1, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5};
char buffer[8];

#else
const uint8_t codes[] = {PS2_KEY_SPACE, PS2_KEY_TAB, PS2_KEY_ESC,
                         PS2_KEY_DELETE, PS2_KEY_F1, PS2_KEY_F2, PS2_KEY_F3,
                         PS2_KEY_F4, PS2_KEY_F5, PS2_KEY_F6, PS2_KEY_F7,
                         PS2_KEY_F8, PS2_KEY_F9, PS2_KEY_F10, PS2_KEY_F11,
                         PS2_KEY_F12};
const char *const keys[] = {" ", "[Tab]", "[ESC]", "[Del]", "[F1]", "[F2]", "[F3]",
                            "[F4]", "[F5]", "[F6]", "[F7]", "[F8]",
                            "[F9]", "[F10]", "[F11]", "[F12]"};
const int8_t sizes[] = {1, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5};
#endif

// initialize the keyboard library with the numbers of the interface pins
PS2KeyAdvanced keyboard;

void intToByte(int intVal, byte byteSet[])
{
  * ((int *) (byteSet + (sizeof(byteSet) - sizeof(int)))) = intVal;
}

int byteToInt(byte byteSet[])
{
  int intVal = * ((int *) (byteSet + (sizeof(byteSet) - sizeof(int))));;
  return intVal;
}

void eepromWriteInt(unsigned int pageAddress, int intVal)
{
  byte byteBuffer[SHORT_BUFFER_LEN];
  intToByte(intVal, byteBuffer);
  eeprom.writeBlock(pageAddress, byteBuffer, SHORT_BUFFER_LEN);
}

int eepromReadInt(unsigned int pageAddress)
{
  // always make the maximum size, just don't use all of it.
  byte testBuffer[LONG_BUFFER_LEN + 1];
  byte byteBuffer[4];
  byteBuffer[4] = '\0';
  eeprom.readBlock(pageAddress, testBuffer, 4);
  int intVal = byteToInt(byteBuffer);
  return intVal;
}

// EdB 09/04/24
void setupMorseduino()
{
  // RS. need the below to end mark
  pinMode(SIGNALPIN, INPUT);     // this reads our key and the tone decoder chip
  pinMode(SCOPEPIN, OUTPUT);     // this reads our key and the tone decoder chip
  digitalWrite(SIGNALPIN, 1);    // turn on internal pull up resistor
  pinMode(LEFT_BUTTON, INPUT);   // this reads left button
  digitalWrite(LEFT_BUTTON, 1);  // turn on internal pull up resistor
  pinMode(RIGHT_BUTTON, INPUT);  // this reads right button
  digitalWrite(RIGHT_BUTTON, 1); // turn on internal pull up resistor

  pinMode(SPEAKERPIN, INPUT);
  digitalWrite(SPEAKERPIN, 1); // turn on internal pull up resistor

  // Find out if a speaker is attached
  int spk = digitalRead(SPEAKERPIN);
  if (spk == 0)
    speaker = true; // The pin will be grounded through the speaker coil

  if (!digitalRead(LEFT_BUTTON))
    speaker = false; // Hold left button down to mute speaker

  pinMode(SPEAKERPIN, OUTPUT); // switch from input to output for our side tone

  if (sideToneIndex > 9)  // EdB: Unsure how the sideToneIndex could ever be greater than 9 and why default to 4 instead of clipping to 9?
    sideToneIndex = 4; // initialize at 440 Hz
  setSideToneFromIndex();

  if (noiseFilter > 8)
    noiseFilter = 4; // initialize at 4 ms
                     // RS. End of what is needed here
  // EdB - Begin
  // SPI.begin(); // The SPI bus is used to set the value of the digital pot 41010
  // delay(1000);
  // pinMode(SLAVESELECTPIN, OUTPUT);
  Wire.begin();
  Wire.setClock(400000);
  bool b = AD1.begin(); // EdB: // Sets up the Digi-Pot 1
  Serial.println(b ? F("AD1:true") : F("AD1:false"));
  Serial.println(AD1.isConnected());

  bool c = AD2.begin(); // EdB: // Sets up the Digi-Pot 2
  Serial.println(c ? F("AD2:true") : F("AD2:false"));
  Serial.println(AD2.isConnected()); // EdB: End AD2
  // EdB - End

  if (ToneSet > 245)
    ToneSet = 122; // initialize this value if it has not been stored yet
  // EdB - Begin
  digitalI2CPotWrite(AD1, ToneSet);
  // EdB - End
}

// EdB 09/04/24
void setupMorseduinoLcd()
{
  // RS. need the below to end mark
  // build LCD graphic character for memu prompt
  // send dadididadah (-..--) to change sidetone to pitch
  lcd.createChar(0, didit);
  lcd.createChar(1, dahdah);

  printHeader();
  if (LCDROWS > 2)
  {
    lcd.setCursor(5, 3);
    lcd.print(VDATE);
    delay(VERSIONDELAY);
    lcd.setCursor(5, 3);
    lcd.print("          ");
    LCDline = 2;
    lcd.setCursor(0, 2);
  }

#ifdef DEBUG
  Serial.begin(9600);
  Serial.println(F("Morseduino 02"));
  Serial.println(VDATE);
  Serial.print(F("Speaker is "));
  if (speaker)
  {
    Serial.println(F("Connected"));
  }
  else
  {
    Serial.println(F("Disconnected"));
  }
#endif
}
// RS. End of what is needed here

// EdB: Adding support for I2C_EEPROM
void setupEEPROM()
{
  Serial.print(F("I2C_EEPROM_VERSION: "));
  Serial.println(I2C_EEPROM_VERSION);

  Wire.begin();

  eeprom.begin();
  if (!eeprom.isConnected())
  {
    Serial.println(F("ERROR: Can't find eeprom. Stopped..."));
    while (1);
  }
  Serial.print(F("isConnected: "));
  Serial.println(eeprom.isConnected());

  Serial.println("\nTEST: determine size");
  start = micros();
  uint32_t size = eeprom.determineSize(true);
  diff = micros() - start;
  Serial.print("TIME: ");
  Serial.println(diff);
  if (size > 0)
  {
    Serial.print("SIZE: ");
    Serial.print(size);
    Serial.println(" Bytes");
  } else if (size == 0)
  {
    Serial.println("WARNING: Can't determine eeprom size");
  }
  else
  {
    Serial.println("ERROR: Can't find eeprom. Stopped...");
    while (1);
  }
}

void setup()
{
  setupMorseduino(); // EdB 09/04/24

  keyboard.begin(DATAPIN, IRQPIN); // Setup keyboard pins
  keyboard.setNoBreak(1);          // No break codes for keys (when key released)
  keyboard.setNoRepeat(1);         // Don't repeat shift ctrl etc

  pinMode(ControlLed, OUTPUT); // Onboard LED that repeats the same morse code as MorseLed
  pinMode(MorseLed, OUTPUT);   // Led that outputs the morse
  pinMode(A1, INPUT);          // photoresistor
  // pinMode(LED_BUILTIN, OUTPUT);
  delay(100);

  Serial.begin(115200); // 9600 is standerd,  115200 is optomal
  while (!Serial)
    ; // wait until serial comes up on Arduino Leonardo or MKR WiFi 1010
  Serial.println();
  Serial.println(__FILE__); // May not need this LIB part
  Serial.print(F("AD5245_LIB_VERSION: "));
  Serial.println(AD5245_LIB_VERSION);
  // EdB: Adding support for I2C_EEPROM
  setupEEPROM();

  delay(50);

  Wire.begin();
  Wire.setClock(400000);
  delay(100);

  // Moved to setupMorseduino()
  // bool b = AD1.begin(); // Sets up the Digi-Pot 1
  // Serial.println(b ? F("true") : F("false"));
  // Serial.println(AD1.isConnected());
  // bool c = AD2.begin(); // Sets up the Digi-Pot 2
  // Serial.println(c ? F("true") : F("false"));
  // Serial.println(AD2.isConnected());

  lcd.init();      // Initialize the lcd
  lcd.backlight(); // Powers ON the back light
  lcd.setCursor(0, 0);
  lcd.clear();

  lcd2.init();      // initialize the lcd
  lcd2.backlight(); // Powers ON the back light
  lcd2.setCursor(0, 0);
  lcd2.clear();

  setupMorseduinoLcd(); // EdB 09/04/24

  // Auto calibrate the photoresistor to ambiant lighting:
  calibrateThreshold(); // ED..!! will not need this anymore
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void getMorse()
{

  // Reading the LDR light value... Will need to be digital now
  val = analogRead(photoResistorPin); // analogRead(photoResistorPin); digitalRead(photoResistorPin);
  if (debugSensor)
  {
    Serial.print(F("Value is :"));
    Serial.print(String(val));
    Serial.print(F(" - Threshold: "));
    Serial.println(threshold);
  }

  ////////////////////
  if (val >= threshold)
  {
    // first "high" of the light
    if (!lightState)
    {
      lightOffLen = millis() - timer;
      timer = millis();
      lightState = true;
      notAnalysed = true;
      endOfTrans = false;
      if (debugTiming)
      {
        Serial.print(F("Length Off: "));
        Serial.print(String(lightOffLen));
        Serial.print(F(" or "));
        Serial.print(String(lightOffLen / timeUnitLength));
        Serial.println(F(" units"));
      }
      lightOffLen = lightOffLen / timeUnitLength; // We count as units of times 1 by timeUnitLength
    }
    if (debug)
    {
      digitalWrite(ControlLed, HIGH);
    }
  }
  else
  {
    // first "low" of the light
    if (lightState)
    {
      lightOnLen = millis() - timer;
      timer = millis();
      lightState = false;
      notAnalysed = true;
      endOfTrans = false;
      if (debugTiming)
      {
        Serial.print(F("Length On: "));
        Serial.print(String(lightOnLen));
        Serial.print(F(" or "));
        Serial.print(String(lightOnLen / timeUnitLength));
        Serial.println(F(" units"));
      }
      lightOnLen = lightOnLen / timeUnitLength; // We count as units of times defined by timeUnitLength
    }
    if (debug)
    {
      digitalWrite(13, LOW);
    }

    // End of Transmission + finish up the last letter
    if (!endOfTrans && ((millis() - timer) / timeUnitLength) >= 10)
    {
      if (debug)
      {
        Serial.println();
        Serial.print(F("--Letter found: "));
#ifdef PS2_REQUIRES_PROGMEM
        Serial.print(String(pgm_read_byte(MorseTree + codePtr)));
#else
        Serial.print(String(MorseTree[codePtr]));
#endif
        Serial.print(F(" -- Code: "));
        Serial.println(String(codePtr));
      }
#ifdef PS2_REQUIRES_PROGMEM
      Serial.print(pgm_read_byte(MorseTree + codePtr));
      lcd2Print((char)pgm_read_byte(MorseTree + codePtr));
#else
      Serial.print(MorseTree[codePtr]);
      lcd2Print((char)MorseTree[codePtr]);
#endif
      codePtr = 0;
      endOfTrans = true;
      if (debug)
      {
        Serial.println(F(" End of Transmission"));
      }
      Serial.println(F(""));
      Serial.println(F("---------------------------------------------------------------"));
    }
  }

  /*** Morse decoding ***/

  // When light is off, we can check if short or long signal was given
  if (notAnalysed && !lightState)
  {
    if (lightOnLen <= 1)
    {
      Serial.print(F("."));
      codePtr = (2 * codePtr) + 1;
      notAnalysed = false;
    }
    else if (lightOnLen >= 2)
    {
      Serial.print(F("-"));
      codePtr = (2 * codePtr) + 2;
      notAnalysed = false;
    }
    else if (lightOnLen >= 3)
    {
      Serial.println(F("********* You stay on too long ************"));
    }
  }

  // When light is on we check for spaces and ends of words or transmission
  if (notAnalysed && lightState)
  {
    if (lightOffLen <= 1)
    {
      // This is a "regular" off between on for now, we do nothing
      notAnalysed = false;
    }
    else if (lightOffLen >= 2 && lightOffLen < 5)
    {
      if (debug)
      {
        Serial.println();
        Serial.print(F("--Letter found: "));
#ifdef PS2_REQUIRES_PROGMEM
        Serial.print(String(pgm_read_byte(MorseTree + codePtr)));
#else
        Serial.print(String(MorseTree[codePtr]));
#endif
        Serial.print(F(" -- Code: "));
        Serial.println(String(codePtr));
      }
#ifdef PS2_REQUIRES_PROGMEM
      Serial.print(pgm_read_byte(MorseTree + codePtr));
      lcd2Print((char)pgm_read_byte(MorseTree + codePtr));
#else
      Serial.print(MorseTree[codePtr]);
      lcd2Print((char)MorseTree[codePtr]);
#endif
      codePtr = 0;
      notAnalysed = false;
    }
    else if (lightOffLen >= 5 && lightOffLen < 10)
    {
      if (debug)
      {
        Serial.println();
        Serial.print(F("--Letter found: "));
#ifdef PS2_REQUIRES_PROGMEM
        Serial.print(pgm_read_byte(MorseTree + codePtr));
#else
        Serial.print(String(MorseTree[codePtr]));
#endif
        Serial.print(F(" -- Code: "));
        Serial.println(String(codePtr));
      }
#ifdef PS2_REQUIRES_PROGMEM
      Serial.print(pgm_read_byte(MorseTree + codePtr));
      lcd2Print((char)pgm_read_byte(MorseTree + codePtr));
#else
      Serial.print(MorseTree[codePtr]);
      lcd2Print((char)MorseTree[codePtr]);
#endif
      codePtr = 0;
      notAnalysed = false;
      if (debug)
      {
        Serial.println(F(" End of Word "));
      }
      Serial.print('#');
      lcd2Print((char)'#');
    }
    else if (lightOffLen > 50)
    {
      // Light was off for so long, this is a new transmission
      lcd2.clear();
      lcd2Pos = 0; // Tracks the position to switch to the second line
    }
  }

  if (debugAdv)
  {
    Serial.print(F("lightOnLen: "));
    Serial.print(String(lightOnLen));
    Serial.print(F(" - lightOffLen: "));
    Serial.print(String(lightOffLen));
    Serial.print(F(" - timeUnitLength: "));
    Serial.print(String(timeUnitLength));
    Serial.print(F(" - Bflag: "));
    Serial.print(String(""));
    Serial.print(F(" - codePtr: "));
    Serial.println(String(codePtr));
  }

  if (debug)
  {
    digitalWrite(13, LOW);
  }
}

/* EdB: Added support for sending morse code from keyboard buffer
 * Send the morse from the global variable, morseCode
 *
 */

void sendMorseCode()
{
  /*******************************************************/
  /* Flash the light according to what is in the buffers */
  /*******************************************************/

  // TODO might be more readable if i put all the different cases in the same if/elseif/else instead of nested ones
  // TODO BAZINGA ACtually, let's switch the damn thing to while loops, at least for the transmitting part...

  // If we do not have anything to transmit in the TX buffer we will read the morse buffer
  if (txBuffer.length() == 0)
  {
    // If we need to transmit morse
    if (morseCode.length() > 0)
    {
      // get the current morse code to process from the morse buffer
      char currentCode = morseCode.charAt(0);

      // H or L corresponds to 1*timeUnitLength of that state (High or Low)
      // S is a start trigger as we start by removing the first state when starting (TODO more elegant solution)
      switch (currentCode)
      {
      case '.':
        txBuffer = "SHL";
        break;
      case '-':
        txBuffer = "SHHHL";
        break;
        // end of letter
      case '#':
        txBuffer = "SLLL";
        break;
        // end of word
      case '%':
        txBuffer = "SLL";
        break;
      }

      // Now that this code is added to txBuffer, let's reset timers & remove it
      morseCode.remove(0, 1);
      ledChangeTimer = millis(); // This will trigger the start of the transmition, see below

      // Displays the switch from one code to the next and new txBuffer content
      if (debugSender)
      {
        Serial.print(F("-- New Code --"));
      }

      // No more morse to transmit!
    }
    else
    {
      sendPressed = false; // EdB: Reset sendPressed flag
      return;
    }

    // There is something in the transmit buffer, let's transmit it!
  }
  else
  {
    unsigned long currentTime = millis();
    // Are we done transmitting the current state?
    if (currentTime > ledChangeTimer)
    {
      if (debugSender)
      {
        Serial.print(F("current: "));
        Serial.print(currentTime);
        Serial.print(F(" - Change: "));
        Serial.println(ledChangeTimer);
      }

      // We are done! Let's switch to the next state
      txBuffer.remove(0, 1);

      // TODO currently high if error, check better currentState
      if (txBuffer.length() > 0)
      {
        char currentState = txBuffer.charAt(0);
        if (debugSender)
        {
          Serial.print(F("currentState: "));
          Serial.println(currentState);
        }

        if (currentState == 'L')
        {
          digitalWrite(MorseLed, LOW);
          digitalWrite(ControlLed, LOW);
          ledChangeTimer = millis() + timeUnitLength;
        }
        else
        {
          digitalWrite(MorseLed, HIGH);
          digitalWrite(ControlLed, HIGH);
          ledChangeTimer = millis() + timeUnitLength;
        }
        if (debugSender)
        {
          Serial.print(F("txBuffer: "));
          Serial.print(txBuffer);
          Serial.print(F(" - morseCode: "));
          Serial.println(morseCode);
        }
      }
      else
      {
        // If nothing to send next, we return and the process will restart at the next loop
        return;
      }
    }
    else
    {
      // we are still in progress on the current state, nothing to do for now
      return;
    }
  }
}

/* EdB: Added support for sending morse code from keybrdBuffer
 * Translate the morse in the keybrdMessage buffer to morseCode
 *
 * Steps:
 * 		- If there is any new message in the keybrdBuffer
 *		- Translates the message to a morse buffer
 *		- Reads the buffers to decide what to send
 */
void translateKeybrdBufferToMorseCode()
{
  if (keybrdBuffer.length() == 0)
    return;
  // Receiving a new message resets the current morse "Bufffer"
  morseCode = "";
  // replacing spaces by _
  keybrdBuffer.replace(" ", "_");
  // Getting rid of caps
  keybrdBuffer.toLowerCase();

  // Translating the message to morse TODO improve this! can do the whole string at once I guess, or include it in translate?
  unsigned int length = keybrdBuffer.length();
  for (unsigned int i = 0; i < length; i++)
  {
    translate(keybrdBuffer.charAt(i)); // TODO pass the char array in parameter
  }

  Serial.print(F("Sending message: "));
  Serial.print(keybrdBuffer);
  Serial.print(F(" - Morse: "));
  Serial.println(morseCode);

  // Now that the message is translated, we add # to detext the end of transmission
  // --> TODO check if it clashes with unknown char, it's late, i changed it three times and lost track lol
  morseCode.concat("#");

  keybrdBuffer = ""; // Reset the keyboard buffer
}

/* Send the morse inputted via Serial
 *
 * Steps:
 * 		- Reads if there is any new message over Serial
 *		- Translates the message to a morse buffer
 *		- Reads the buffers to decide what to send
 */
void sendMorseFromSerial()
{
  // will need code here to read a buffer from the keyboard to transment
  /**************************************************************/
  /* Read the message fronm serial + translate it to morse code */
  /**************************************************************/
  while (Serial.available())
  {

    // Receiving a new message resets the current morse "Bufffer"
    morseCode = "";
    message = Serial.readStringUntil('\n');
    // replacing spaces by _
    message.replace(" ", "_");
    // Getting rid of caps
    message.toLowerCase();

    // Reading one letter example
    // serialReceived.substring(0,1).toCharArray(commandChar, 2);

    // Translating the message to morse TODO improve this! can do the whole string at once I guess, or include it in translate?
    unsigned int length = message.length();
    for (unsigned int i = 0; i < length; i++)
    {
      translate(message.charAt(i)); // TODO pass the char array in parameter
    }

    Serial.print(F("Sending message: "));
    Serial.print(message);
    Serial.print(F(" - Morse: "));
    Serial.println(morseCode);

    // Now that the message is translated, we add # to detext the end of transmission
    // --> TODO check if it clashes with unknown char, it's late, i changed it three times and lost track lol
    morseCode.concat("#");
  }

  sendMorseCode(); // EdB: Moved common code to this new function
}

// threshold value for the photoresistor to differenciate between (light on) and (light off)
void calibrateThreshold()
{
  int initValue = analogRead(photoResistorPin);
  threshold = initValue * 1.25; // Threshoold is 25% more than the current reading of light.
  if (threshold < 250)
  {
    threshold = 250;
  } // Low light tweak
    // TODO remember max seen and lowest seen and take middle
}

void lcdPrint(char c)
{
  // Change line on the 1st character
  if (lcdPos == 0)
  {
    lcd.setCursor(0, 0);
  }

  // Change line on the 20th character to line 2
  if (lcdPos == 20)
  {
    lcd.setCursor(0, 1);
  }

  // Change line on the 40th character to line 3
  if (lcdPos == 40)
  {
    lcd.setCursor(0, 2);
  }

  // Change line on the 60th character to line 4
  if (lcdPos == 60)
  {
    lcd.setCursor(0, 3);
  }

  // Message too long, clear and back to first line
  if (lcdPos == 80)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcdPos = 0;
  }

  // Display the character!
  lcd.print(c);
  lcdPos++;
}

void lcd2Print(char c)
{
  // Change line on the 1st character
  if (lcd2Pos == 0)
  {
    lcd2.setCursor(0, 0);
  }

  // Change line on the 20th character to line 2
  if (lcd2Pos == 20)
  {
    lcd2.setCursor(0, 1);
  }

  // Change line on the 40th character to line 3
  if (lcd2Pos == 40)
  {
    lcd2.setCursor(0, 2);
  }

  // Change line on the 60th character to line 4
  if (lcd2Pos == 60)
  {
    lcd2.setCursor(0, 3);
  }

  // Message too long, clear and back to first line
  if (lcd2Pos == 80)
  {
    lcd2.clear();
    lcd2.setCursor(0, 0);
    lcd2Pos = 0;
  }

  // Display the character!
  lcd2.print(c);
  lcd2Pos++;
}

String translate(char *i)
{

  // Take the passed character and use a switch case to find the morse code for that character
  switch ((char)i)
  {
  case 'a':
    morseCode += ".-";
    break;
  case 'b':
    morseCode += "-...";
    break;
  case 'c':
    morseCode += "-.-.";
    break;
  case 'd':
    morseCode += "-..";
    break;
  case 'e':
    morseCode += ".";
    break;
  case 'f':
    morseCode += "..-.";
    break;
  case 'g':
    morseCode += "--.";
    break;
  case 'h':
    morseCode += "....";
    break;
  case 'i':
    morseCode += "..";
    break;
  case 'j':
    morseCode += ".---";
    break;
  case 'k':
    morseCode += "-.-";
    break;
  case 'l':
    morseCode += ".-..";
    break;
  case 'm':
    morseCode += "--";
    break;
  case 'n':
    morseCode += "-.";
    break;
  case 'o':
    morseCode += "---";
    break;
  case 'p':
    morseCode += ".--.";
    break;
  case 'q':
    morseCode += "--.-";
    break;
  case 'r':
    morseCode += ".-.";
    break;
  case 's':
    morseCode += "...";
    break;
  case 't':
    morseCode += "-";
    break;
  case 'u':
    morseCode += "..-";
    break;
  case 'v':
    morseCode += "...-";
    break;
  case 'w':
    morseCode += ".--";
    break;
  case 'x':
    morseCode += "-..-";
    break;
  case 'y':
    morseCode += "-.--";
    break;
  case 'z':
    morseCode += "--..";
    break;
  case '.':
    morseCode += ".-.-.-";
    break;
  case '-':
    morseCode += "-....-";
    break;
  case '(':
    morseCode += "-.--.";
    break;
  case ')':
    morseCode += "-.--.-";
    break;
  case '@':
    morseCode += ".--.-.";
    break;
  case '$':
    morseCode += "...-..-";
    break;
  case '!':
    morseCode += "-.-.--";
    break;
  case '_':
    // space -> end of word
    morseCode += "..--.-";
  default:
    // unrecognised character
    morseCode += "@";
    break;
  }

  // To identify the end of each letter, we apped a # ()
  morseCode += "#";
  return (morseCode); // BAZINGA TODO WRONG JUST A TEST
}

// TEMP Manual morse button sender
// char readBtn()
// {
//   if (signal_len < 500 && signal_len > 50)
//   {
//     return '.';                        //if button press less than 0.6sec, it is a dot
//   }
//   else if (signal_len > 500)
//   {
//     return '-';                        //if button press more than 0.6sec, it is a dash
//   }
// }

void convertor()
{
  static String letters[] = {".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..", ".---", "-.-", ".-..", "--", "-.", "---", ".--.", "--.-",
                             ".-.", "...", "-", "..-", "...-", ".--", "-..-", "-.--", "--..", "E"};
  int i = 0;
  if (code == ".-.-.-")
  {
    Serial.print(F(".")); // for break
  }
  else
  {
    while (letters[i] != "E") // loop for comparing input code with letters array
    {
      if (letters[i] == code)
      {
        Serial.print(char('A' + i));
        break;
      }
      i++;
    }
    if (letters[i] == "E")
    {
      Serial.println(""); // if input code doesn't match any letter, error
    }
  }
  code = ""; // reset code to blank string
}

void thresholdPlotter()
{
  int y1 = analogRead(photoResistorPin);
  int y2 = threshold;
  int y3 = 0; // by default we assume the information is 0
  if (y1 > y2)
  {
    y3 = 1024;
  } // if value > threshold, we switch it to 1

  Serial.print(y1);
  Serial.print(" "); // a space ' ' or  tab '\t' character is printed between the two values.
  Serial.print(y2);
  Serial.print(" "); // a space ' ' or  tab '\t' character is printed between the two values.
  Serial.println(y3);

  delay(100);
}

// RS. need the below to end mark
///////////////////////////////////////////////////////////////////
// This method sends the value to the I2C digital pot
// Parameter: 0-254
int digitalI2CPotWrite(AD5245 ad, int value)
{
  ad.write(value);
}

/////////////////////////////////////////////////////////////////////////////////
// This method prints Splash text on LCD
void printHeader()
{
  lcd.clear(); // Get rid of any garbage that might appear on startup
  // delay(2000);
  lcd.print(HISCALL);
  // lcd.setCursor(3,0);
  lcd.print(" MORSEDUINO 02");
  lcd.setCursor(0, 1);
  lcd.print("TS=");
  lcd.print(ToneSet);
  lcd.print(" QRNf=");
  lcd.print(noiseFilter);
  lcd.print(" SPK=");
  lcd.print(speaker); // 1=speaker connected, 0=speaker disconnected
  lcd.setCursor(0, LCDROWS - 2);
  justDid = true; // so we don't start with a space at the beginning of the line
}
// RS. End of what is needed here

void loop()
{
  // RS. need the below to end mark
  signalPinState = digitalRead(SIGNALPIN); // What is the tone decoder doing?
  if (!signalPinState)
    keyIsDown(); // LOW, or 0, means tone is being decoded
  if (signalPinState)
    keyIsUp(); // HIGH, or 1, means no tone is there

  // Now check to see if a buttons is pressed
  if (!digitalRead(LEFT_BUTTON))
    changePitch();
  if (!digitalRead(RIGHT_BUTTON))
    checkRIGHT_BUTTON();

  // Wait 1.5 seconds before writing changes to EEPROM
  // lastChange will be 0 if no buttons have been pressed
  if (lastChange)
  {
    if (millis() - lastChange > 1500)
      resetDefaults();
  }
  // RS. End of what is needed here

  if (Serial.available() > 0)
  {
    byte incomingByte;
    // Read the incoming byte from Serial Input:
    incomingByte = Serial.read();
    // Serial.print("incomingByte: "); Serial.println(incomingByte);

    // Limited to 93 ascii codes:
    if ((incomingByte >= '!') and (incomingByte <= ('~')))
    {
      ComCommand = incomingByte;
      Serial.println("");
      Serial.print(F("Response: "));
      Serial.println(ComCommand);
      Serial.println("");
      ComInput();
    }
  }
  Serial.println(note);
  tone(audio, note);
  // delay(1000);
  // noTone(audio);
  //  makes note increse
  /*if (note <= 1000; note++) {
    Serial.println(note);
    tone(audio, note);
    //delay(500);
    //noTone(audio);
  }
  if (note > 1000) {
    note = 400;
  }*/

  Serial.println(" ");
  // val2 < 255; val2++;

  // AD2 adjusts the volume of the output tone in relation to the Frequency being outputted.
  // low Feq 400Hz is a higher Volume, so the amp needs to adjust down.
  // A higher Feq 1200Hz is a lower volume, so the amp needs to adjust up,
  // to hold each Feq at 5V peak to peak max output to the radio's mic input
  Serial.print(val2);
  AD2.write(val2);
  delay(50);

  Serial.print('\t');
  Serial.print(AD2.read());
  delay(10);

  Serial.print('\t');
  Serial.println(AD2.readDevice());
  delay(500);

  keyBrd(); // call keyboard
  // If we activate the serial plotter, we deactivate everything else
  // TODO only deactivate any other serial communication!
  if (debugPlotter)
  {
    thresholdPlotter();
  }
  else
  {
    // EdB: Start new code section - Support sending morese from keybrdBuffer
    if (sendPressed)
    {
      translateKeybrdBufferToMorseCode();
      sendMorseCode(); // Called mulitple times until the entire morseCode is sent
    }
    // will send morse thru the MorseLed based on Serial input.
    // sendMorseFromSerial();
    // EdB: End new code section

    // Reads the incoming morse code thru the photoresistor
    getMorse();
  }
}

// RS. need the below to end mark
void checkRIGHT_BUTTON()
{
  if (sideToneSet)
  {
    // We also use this button to change the side tone if a speaker is attached
    sideToneIndex++;
    setSideTone();
    return;
  }

  bumpFilter();
  delay(500);
  // if button is held down start the auto sweep routine
  if (!digitalRead(RIGHT_BUTTON))
  {
    noiseFilter--; // undo the filter bump
    if (noiseFilter < 0)
      noiseFilter = 0;
    sweep();
  }
}

void bumpFilter()
{
  oldNoiseFilter = noiseFilter;
  noiseFilter++;
  if (noiseFilter > 8)
    noiseFilter = 0; // wrap value around
  lcd.clear();
  lcd.print("QRN Filter: ");
  lcd.print(noiseFilter);

  // reset the scroll and word wrap variables
  LCDline = 1;
  lcd.setCursor(0, 1);
  lastSpace = 0;     // clear the last space pointer
  lastWordCount = 0; // clear the last word length
  letterCount = 0;

  // reset the adjust speed variables
  downTime = 0;
  upTime = 0;
  dit = 10;         // We start by defining a dit as 10 milliseconds
  averageDah = 100; // A dah should be 3 times as long as a dit
  fullWait = 6000;  // The time between letters
  waitWait = 6000;
  myMax = 0;
  myMin = 255;
}

void resetDefaults()
{
  printHeader();

  lastSpace = 0;     // clear the last space pointer
  lastWordCount = 0; // clear the last word length
  letterCount = 0;
  downTime = 0;
  upTime = 0;
  dit = 10;         // We start by defining a dit as 10 milliseconds
  averageDah = 100; // A dah should be 3 times as long as a dit
  fullWait = 6000;  // The time between letters
  waitWait = 6000;
  myMax = 0;
  myMin = 255;
  LCDline = 2;

  // only write to EEPROM if value has changed
  if (oldToneSet != ToneSet) // RS. may need to change to match new EEPROM
    eepromWriteInt(addr, ToneSet);
  if (oldNoiseFilter != noiseFilter)
    eepromWriteInt(addr + 1, noiseFilter);
  lastChange = 0; // turn timer off until next changes are made

  oldToneSet = ToneSet;
  oldNoiseFilter = noiseFilter;
}

void changePitch()
{
  if (sideToneSet)
  {
    sideToneIndex--;
    setSideTone();
    return;
  }

  delay(200); // autorepeat

  lastChange = millis(); // reset timer so we know to save change later
  oldToneSet = ToneSet;
  // if it has been more than 1 second since this button was pressed reverse the
  // direction of the pitch change
  if (millis() - pitchTimer > 1000)
  {
    pitchDir = pitchDir * -1;
  }
  pitchTimer = millis();
  ToneSet = ToneSet + pitchDir;
  if (ToneSet > 255)
    ToneSet = 255;
  if (ToneSet < 0)
    ToneSet = 0;

  if (oldToneSet == ToneSet)
    return;

  // EdB - Start
  digitalI2CPotWrite(AD1, ToneSet);
  // EdB - End
  lcd.setCursor(0, LCDROWS - 1);
  lcd.print(ToneSet); // show us the new value
  lcd.print("    ");
}

void sweep()
{
  sweepCount = 0;
  oldToneSet = ToneSet;
  lcd.clear();
  lcd.print("Sweep");
  myMin = 255;
  myMax = 0;

  // We keep calling the sweep methods as long as our
  // best minimum is greater than our best maximum
  while (myMin > myMax)
  {
    sweepCount = sweepCount + SWEEPCOUNTBUMP; // used to create a delay that increases
                                              // with each pass
#ifdef DEBUG
    Serial.print(F("\nsweepCount="));
    Serial.println(sweepCount);
#endif

    sweepUp();
    sweepDown();
  }
  // Find a value that is half way between the min and max
  ToneSet = (myMax - myMin) / 2;
  ToneSet = ToneSet + myMin;
  // EdB - Start
  digitalI2CPotWrite(AD1, ToneSet); // send this value to the digital pot
  // EdB - End
#ifdef DEBUG
  Serial.print(F("\nTone Parked at:"));
  Serial.println(ToneSet);
#endif
  lcd.clear();
  resetDefaults(); // store the changes right away don't wait
}

void sweepUp()
{

#ifdef DEBUG
  Serial.println(F("\nSweep Up"));
#endif

  lcd.clear();
  lcd.print("Sweep Up");
  // lcd.setCursor(8,0);

  for (ToneSet = 0; ToneSet <= MAXSWEEP; ToneSet += 2)
  {
    // EdB - Start
    digitalI2CPotWrite(AD1, ToneSet);
    // EdB - End
    delay(sweepCount); // this delay gets longer with each pass
    if (!digitalRead(8))
    {
      if (ToneSet > myMax)
      {
        myMax = ToneSet;
#ifdef DEBUG
        Serial.print(F("myMax="));
        Serial.println(myMax);
#endif
        lcd.print('.'); // show our hits on the display
      }
    }
  }
}

void sweepDown()
{
#ifdef DEBUG
  Serial.println(F("\nSweep Down"));
#endif

  lcd.clear();
  lcd.print("Sweep Down");
  // lcd.setCursor(10,0);

  for (ToneSet = MAXSWEEP; ToneSet >= 0; ToneSet -= 2)
  {
    // EdB - Start
    digitalI2CPotWrite(AD1, ToneSet);
    // EdB - End
    delay(sweepCount);
    if (!digitalRead(8))
    {
      if (ToneSet < myMin)
      {
        myMin = ToneSet;
#ifdef DEBUG
        Serial.print(F("myMin="));
        Serial.println(myMin);
#endif
        lcd.print('.');
      }
    }
  }
}

void keyIsDown()
{
  // The LEDs on the decoder and Arduino will blink on in unison
  // digitalWrite(13,1);            // turn on Arduino's LED
  if (noiseFilter)
    delay(noiseFilter);
  signalPinState = digitalRead(SIGNALPIN); // What is the tone decoder doing?
  if (signalPinState)
    return;

  digitalWrite(SCOPEPIN, 0);
  if (speaker)
    tone(SPEAKERPIN, sideTone);
  if (startUpTime > 0)
  {
    // We only need to do once, when the key first goes down
    startUpTime = 0; // clear the 'Key Up' timer
  }
  // If we haven't already started our timer, do it now
  if (startDownTime == 0)
  {
    startDownTime = millis(); // get Arduino's current clock time
  }

  characterDone = false; // we're still building a character
  ditOrDah = false;      // the key is still down we're not done with the tone
  delay(myBounce);       // Take a short breath here

  if (myNum == 0)
  {            // myNum will equal zero at the beginning of a character
    myNum = 1; // This is our start bit  - it only does this once per letter
  }
}

void keyIsUp()
{
  // The LED on the Arduino will blink on and off with the code
  // digitalWrite(13,0);    // turn off Arduino's LED
  if (noiseFilter)
    delay(noiseFilter);
  signalPinState = digitalRead(SIGNALPIN); // What is the tone decoder doing?
  if (!signalPinState)
    return;

  // If we haven't already started our timer, do it now
  if (startUpTime == 0)
  {
    startUpTime = millis();
  }

  // Find out how long we've gone with no tone
  // If it is twice as long as a dah print a space

  upTime = millis() - startUpTime;

  if (upTime < 20)
    return;

  if (speaker)
    noTone(SPEAKERPIN);
  digitalWrite(SCOPEPIN, 1);

  if (upTime > (averageDah * 2))
  {
    printSpace();
  }

  // Only do this once after the key goes up
  if (startDownTime > 0)
  {
    downTime = millis() - startDownTime; // how long was the tone on?
    startDownTime = 0;                   // clear the 'Key Down' timer
  }

  if (!ditOrDah)
  {
    // We don't know if it was a dit or a dah yet
    shiftBits(); // let's go find out! And do our Magic with the bits
  }

  // If we are still building a character ...
  if (!characterDone)
  {
    // Are we done yet?
    if (upTime > dit)
    {
      // BINGO! we're done with this one
      printCharacter();     // Go figure out what character it was and print it
      characterDone = true; // We got him, we're done here
      myNum = 0;            // This sets us up for getting the next start bit
    }
    downTime = 0; // Reset our keyDown counter
  }
}

void shiftBits()
{
  // we know we've got a dit or a dah, let's find out which
  // then we will shift the bits in myNum and then add 1 or not add 1

  if (downTime < dit / 3)
    return;           // ignore my keybounce
  myNum = myNum << 1; // shift bits left
  ditOrDah = true;    // we will know which one in two lines

  // If it is a dit we add 1. If it is a dah we do nothing!
  if (downTime < dit)
  {
    myNum++; // add one because it is a dit
  }
  else
  {

    // The next three lines handle the automatic speed adjustment:
    averageDah = (downTime + averageDah) / 2; // running average of dahs
    dit = averageDah / 3;
    // normal dit would be this
    dit = dit * 2; // double it to get the threshold between dits and dahs
  }
}

void setSideToneFromIndex()
{
  if (sideToneIndex < 1)
    sideToneIndex = 1;
  if (sideToneIndex > 9)
    sideToneIndex = 9;
  sideTone = sideToneIndex * 110;
}

void setSideTone()
{
  // Send dadididada ( -..--) to enter and exit this mode
  if (!sideToneSet)
    lcd.clear();
  lcd.setCursor(0, 0);
  delay(200);
  setSideToneFromIndex();
  lcd.print("SIDE TONE FREQ:");
  lcd.print(sideTone);
  lcd.print("Hz");
  lcd.setCursor(1, 1);
  lcd.print("Send: ");
  lcd.write(byte(1));
  lcd.write(byte(0));
  lcd.write(byte(1));
  lcd.write(byte(1));
  lcd.print(" to exit");
  lcd.setCursor(0, 2);
  lcd.print("LT/RT = LOWER/HIGHER");
  lcd.setCursor(0, 3);
  LCDline = 3;
  justDid = true;
  sideToneSet = true;
  tone(SPEAKERPIN, sideTone);
  delay(100);
  noTone(speaker);
}

void printCharacter()
{

  justDid = false; // OK to print a space again after this

  if (myNum == 44 && speaker)
  {
    if (!sideToneSet)
    {
      setSideTone();
    }
    else
    {
      sideToneSet = false;
      eepromWriteInt(addr + 2, sideToneIndex);

      resetDefaults();
    }
    return;
  }

  // Punctuation marks will make a BIG myNum
  if (myNum > 63)
  {
    printPunctuation(); // The value we parsed is bigger than our character array
                        // It is probably a punctuation mark so go figure it out.
    return;             // Go back to the main loop(), we're done here.
  }
  lcdGuy = mySet[myNum]; // Find the letter in the character set
  sendToLCD();           // Go figure out where to put in on the display
}

void printSpace()
{
  if (justDid)
    return;       // only one space, no matter how long the gap
  justDid = true; // so we don't do this twice
  // Farns = 2 + !digitalRead(farnsRead)*2;

  lastWordCount = 0;              // start counting length of word again
  currentLine[letterCount] = ' '; // add a space to the variable that stores the current line
  lastSpace = letterCount;        // keep track of this, our last, space

  // Now we need to clear all the characters out of our last word array
  for (int i = 0; i < 20; i++)
  {
    lastWord[i] = ' ';
  }

  lcdGuy = ' '; // this is going to go to the LCD

  // We don't need to print the space if we are at the very end of the line
  if (letterCount < 20)
  {
    sendToLCD(); // go figure out where to put it on the display
  }
}
// RS. End of what is needed here

// RS. the below Punctuation part mey not be needed
// EdB: printPunctuation() is being called from printCharacter() so if it remains there, it is needed here.
void printPunctuation()
{
  // Punctuation marks are made up of more dits and dahs than
  // letters and numbers. Rather than extend the character array
  // out to reach these higher numbers we will simply check for
  // them here. This function only gets called when myNum is greater than 63

  // Thanks to Jack Purdum for the changes in this function
  // The original uses if then statements and only had 3 punctuation
  // marks. Then as I was copying code off of web sites I added
  // characters we don't normally see on the air and the list got
  // a little long. Using 'switch' to handle them is much better.

  switch (myNum)
  {
  case 71:
    lcdGuy = ':';
    break;
  case 76:
    lcdGuy = ',';
    break;
  case 84:
    lcdGuy = '!';
    break;
  case 94:
    lcdGuy = '-';
    break;
  case 97:
    lcdGuy = 39; // Apostrophe
    break;
  case 101:
    lcdGuy = '@';
    break;
  case 106:
    lcdGuy = '.';
    break;
  case 115:
    lcdGuy = '?';
    break;
  case 246:
    lcdGuy = '$';
    break;
  case 122:
    lcdGuy = 's';
    sendToLCD();
    lcdGuy = 'k';
    break;
  default:
    lcdGuy = '#'; // Should not get here
    break;
  }
  sendToLCD(); // go figure out where to put it on the display
}

void sendToLCD()
{
  // Do this only if the character is a 'space'
  if (lcdGuy > ' ')
  {
    lastWord[lastWordCount] = lcdGuy; // store the space at the end of the array
    if (lastWordCount < lineEnd - 1)
    {
      lastWordCount++; // only bump up the counter if we haven't reached the end of the line
    }
  }
  currentLine[letterCount] = lcdGuy; // now store the character in our current line array

  letterCount++; // we're counting the number of characters on the line
  if (letterCount == lineEnd - 5)
    clearSpeed();
  if (letterCount > 5)
  {
    // to extend EEPROM life we will only write when a charcter is printed and the value has changed.
    if (noiseFilter != oldNoiseFilter)
    {
      eepromWriteInt(addr + 1, noiseFilter);
      oldNoiseFilter = noiseFilter;
      // Serial.println("noiseFilter saved to EEPROM");
    }
  }

  // If we have reached the end of the line we will go do some chores
  if (letterCount == lineEnd)
  {
    newLine(); // check for word wrap and get ready for the next line
    return;    // so we don't need to do anything more here
  }

  lcd.print(lcdGuy); // print our character at the current cursor location
}

//////////////////////////////////////////////////////////////////////////////////////////
// The following functions handle word wrapping and line scrolling for a 4 line display //
//////////////////////////////////////////////////////////////////////////////////////////
// RS. the other LCD code may be needed here
void newLine()
{
  // sendToLCD() will call this routine when we reach the end of the line
  if (lastSpace == 0)
  {
    // We just printed an entire line without any spaces in it.
    // We cannot word wrap this one so this character has to go at
    // the beginning of the next line.

    // First we need to clear all the characters out of our last word array
    for (int i = 0; i < 20; i++)
    {
      lastWord[i] = ' ';
    }

    lastWord[0] = lcdGuy; // store this character in the first position of our next word
    lastWordCount = 1;    // set the length to 1
  }
  truncateOverFlow(); // Trim off the first part of a word that needs to go on the next line

  linePrep();        // Store the current line so we can move it up later
  reprintOverFlow(); // Print the truncated text and space padding on the next line
  printHisSpeed();
}

void clearSpeed()
{
  lcd.setCursor(lineEnd - 3, 3);
  lcd.print("  ");
  lcd.setCursor(letterCount - 1, LCDline);
}

// RS. need the below to end mark
void printHisSpeed()
{
  lcd.setCursor(lineEnd - 3, 3);
  wpm = dit / 2;
  wpm = 1400 / wpm;
  // wpm = wpm + wpm * .08);
  lcd.print(wpm);
  lcd.setCursor(letterCount, LCDline);
}
// RS. End of what is needed here

void truncateOverFlow()
{
  // Our word is running off the end of the line so we will
  // chop it off at the last space and put it at the beginning of the next line

  if (lastSpace == 0)
  {
    return;
  } // Don't do this if there was no space in the last line

  // Move the cursor to the place where the last space was printed on the current line
  lcd.setCursor(lastSpace, LCDline);

  letterCount = lastSpace; // Change the letter count to this new shorter length

  // Print 'spaces' over the top of all the letters we don't want here any more
  for (int i = lastSpace; i < 20; i++)
  {
    lcd.print(' ');       // This space goes on the display
    currentLine[i] = ' '; // This space goes in our array
  }
}

void linePrep()
{
  LCDline++; // This is our line number, we make it one higher

  // What we do next depends on which line we are moving to
  // The first three cases are pretty simple because we working on a cleared
  // screen. When we get to the bottom, though, we need to do more.
  switch (LCDline)
  {
  case 1:
    // We just finished line 0
    // don't need to do anything because this for the top line
    // it is going to be thrown out when we scroll anyway.
    break;
  case 2:
    // We just finished line 1
    // We are going to move the contents of our current line into the line1 array
    for (int j = 0; j < 20; j++)
    {
      line1[j] = currentLine[j];
    }
    break;
  case 3:
    // We just finished line 2
    // We are going to move the contents of our current line into the line2 holding bin
    for (int j = 0; j < 20; j++)
    {
      line2[j] = currentLine[j];
    }
    break;
  case 4:
    // We just finished line 3
    // We are going to move the contents of our current line into the line3 holding bin
    for (int j = 0; j < 20; j++)
    {
      line3[j] = currentLine[j];
    }
    // This is our bottom line so we will keep coming back here
    LCDline = 3; // repeat this line over and over now. There is no such thing as line 4

    myScroll(); // move everything up a line so we can do the bottom one again
    break;
  }
}

void myScroll()
{
  // We will move each line of text up one row

  int i = 0; // we will use this variables in all our for loops

  lcd.setCursor(0, 0); // Move the cursor to the top left corner of the display
  lcd.print(line1);    // Print line1 here. Line1 is our second line,
                       // our top line is line0 ... on the next scroll
                       // we toss this away so we don't store line0 anywhere

  // Move everything stored in our line2 array into our line1 array
  for (i = 0; i < 20; i++)
  {
    line1[i] = line2[i];
  }

  lcd.setCursor(0, 1); // Move the cursor to the beginning of the second line
  lcd.print(line1);    // Print the new line1 here

  // Move everything stored in our line3 array into our line2 array
  for (i = 0; i < 20; i++)
  {
    line2[i] = line3[i];
  }
  lcd.setCursor(0, 2); // Move the cursor to the beginning of the third line
  lcd.print(line2);    // Print the new line2 here

  // Move everything stored in our currentLine array into our line3 array
  for (i = 0; i < 20; i++)
  {
    line3[i] = currentLine[i];
  }
}

void reprintOverFlow()
{
  // Here we put the word that wouldn't fit at the end of the previous line
  // Back on the display at the beginning of the new line

  // Load up our current line array with what we have so far
  for (int i = 0; i < 20; i++)
  {
    currentLine[i] = lastWord[i];
  }

  lcd.setCursor(0, LCDline);   // Move the cursor to the beginning of our new line
  lcd.print(lastWord);         // Print the stuff we just took off the previous line
  letterCount = lastWordCount; // Set up our character counter to match the text
  lcd.setCursor(letterCount, LCDline);
  lastSpace = 0;     // clear the last space pointer
  lastWordCount = 0; // clear the last word length
}

// TODO Stop using Strings everywhere and switch to cstrings!
// If the arduino freezes it might be because of too much memory gruyere from the String usage

//................... Keyboard Section .........

// Check if cursor now moved beyond end of column, or to the last row
void check_cursor()
{
  if (cols >= MAX_COL)
  { // if colums is more than max change rows
    cols = 0;
    rows++;
    if (rows >= MAX_ROW) // if rows are more than max go back to to first row
      rows = 0;
    lcd.setCursor(cols, rows);
    if (cols == 0 && rows == 0) // if max columns and max rows reached,
      lcd.clear();              // clear the LCD Text and start over at 0-0
  }
}

void keyBrd()
{
  // mode = 0 echo character
  // mode = 1 print string
  // mode = 2 cursor movement NO other echo
  // mode = 4 ignore key no echo
  byte mode = 0;
  byte idx = 0;
  int c = 0;

  if (keyboard.available())
  {
    // read the next key
    if ((c = keyboard.read()))
    {
      // check for some of the special keys
      mode = 2;
      c &= 0xFF;
      switch (c) // Cursor movements
      {
      case PS2_KEY_ENTER: // Cursor to beginning of next line or start..... need a send morse here
      case PS2_KEY_KP_ENTER:
        sendPressed = true; // EdB: Support sending morse code from keyboard buffer when the <Enter> (send) key is pressed
        cols = 0;
        // EdB: Don't move to next line. Clear the LCD display instead.
        // rows++;
        // if (rows >= MAX_ROW)
        //   rows = 0;
        lcd.clear(); // EdB: Clear the LCD display
        break;
      case PS2_KEY_PGDN: // Cursor to top row current column
        rows = MAX_ROW - 1;
        break;
      case PS2_KEY_PGUP: // Cursor to bottom row current column
        rows = 0;
        break;
      case PS2_KEY_L_ARROW: // Cursor left or end of previous line
        cols--;
        if (cols < 0)
        {
          cols = MAX_COL - 1;
          rows--;
          if (rows < 0)
            rows = MAX_ROW - 1;
        }
        break;
      case PS2_KEY_R_ARROW: // Cursor right or start of next line
        cols++;
        if (cols >= MAX_COL)
        {
          cols = 0;
          rows++;
          if (rows >= MAX_COL)
            rows = 0;
        }
        break;
      case PS2_KEY_UP_ARROW: // Cursor up one line no wrap
        rows--;
        if (rows < 0)
          rows = 0;
        break;
      case PS2_KEY_DN_ARROW: // Cursor down one line no wrap
        rows++;
        if (rows >= MAX_ROW)
          rows = MAX_ROW - 1;
        break;
      case PS2_KEY_BS: // Move cursor back write space move cursor back
        cols--;
        if (cols < 0)
        {
          cols = MAX_COL - 1;
          rows--;
          if (rows < 0)
            rows = MAX_ROW - 1;
        }
        lcd.setCursor(cols, rows);
        lcd.write(' ');
        break;
      case PS2_KEY_HOME: // Cursor to top left
        cols = 0;
        rows = 0;
        break;
      case PS2_KEY_END: // Cursor to max position
        cols = MAX_COL - 1;
        rows = MAX_ROW - 1;
        break;
      default: // Not cursor movement
        mode = 0;
      }
      // if was cursor movement do last movement
      if (mode == 2)
        lcd.setCursor(cols, rows);
      else
      {
        // Check for strings or single character to display
        // Function or similar key
        if (c != PS2_KEY_EUROPE2 && (c < PS2_KEY_KP0 || c >= PS2_KEY_F1))
        { // Non printable sort which ones we can print
          for (idx = 0; idx < sizeof(codes); idx++)
#ifdef PS2_REQUIRES_PROGMEM
            if (c == pgm_read_byte(codes + idx))
#else
            if (c == codes[idx])
#endif
            { // String outputs
              mode = 1;
#ifdef PS2_REQUIRES_PROGMEM
              c = pgm_read_byte(sizes + idx);
#else
              c = sizes[idx];
#endif
              cols += c - 1;
              check_cursor();
              // when cursor reset keep track
              if (cols == 0)
                cols = c;
#ifdef PS2_REQUIRES_PROGMEM
              strcpy_P(buffer, (char *)pgm_read_word(&(keys[idx])));
              lcd.print(buffer);
#else
              lcd.print(keys[idx]);
#endif
              cols++;
              check_cursor();
              break;
            }
          // if not found a string ignore key cant do anything
        }
        else
        { // Supported key
          if (c <= 127 || c > 0)
          {
            // EdB: Start code section - Added support for saving printable keyboard characters to a buffer to be sent when the <Enter> (send) key is pressed
            if (c > 31)
            {
              keybrdBuffer += (char)c; // Printable characters are from ASCII decimal 32 to 126
            }
            // EdB: End code section
            check_cursor();
            cols++;
            lcd.write(c);
            check_cursor();
          }
        }
      }
    }
    delay(100);
  }
}

///////////////////// Serial Input Command Section ///////////////////
int ComInput()
{
  switch (ComCommand)
  {
  // The Serial Port tells this section what to print or do
  // after receving a command

  // case '0':  // ComCommand = 0 = 400Hz
  case 48: // Return input = 0
    Serial.println(F("Frequance = 400Hz"));
    note = 400;
    break;

  // case '1':  // ComCommand = 1 = 1000Hz
  case 49:
    Serial.println(F("Frequance = 1000Hz"));
    note = 1000;
    break;

  // case '2':  // ComCommand = 2
  case 50: // Return input 2
    Serial.println(F("Increse Resistance + 1"));
    if (val2 < 255)
    {
      val2++;
      Serial.println(val2);
    }
    break;

  // case '3': // ComCommand = 3
  case 51: // Return input 3
    Serial.println(F("Reduce Resistance - 1"));
    if (val2 > 0)
    {
      val2--;
      Serial.println(val2);
    }
    // 400Hz = 95 = 4.961V  47K R
    // 1000Hz = 249 = 4.9161V 47K R
    break;

  // case '4':  // ComCommand = 4
  case 52: // Return input 4
    Serial.println(F("Increse Resistance + 10"));
    val2 = val2 + 10;
    break;

  // case '5':  // ComCommand = 5
  case 53: // cReturn input 5
    Serial.println(F("Reduce Resistance - 10"));
    val2 = val2 - 10;
    break;

  // case '6': // ComCommand = 6
  case 54: // Return input 6
    Serial.println(F("Case = 6"));
    Serial.println(F("Frequance = 400Hz Val2 = 65"));
    note = 400;
    val2 = 65;
    break;

  // case '7': // ComCommand = 7
  case 55: // Return input 7
    Serial.println(F("Frequance = 1000Hz Val2 = 158"));
    note = 1000;
    val2 = 158;
    break;

  // case '8':  // ComCommand = 8
  case 56: // Return input 8
    Serial.println(F("Case = 8"));
    break;

  // case '9': // ComCommand = 9
  case 57: // Return input 9
    Serial.println(F("Case = 9"));
    break;
    //"""""""""""""""""""""""" Letters """""""""""""""""""""""""""""""""
  case 97: // ComCommand a
    Serial.println(F("Frequance = 400Hz"));
    note = 400;
    val2 = 65;
    break;

  case 98: // ComCommand b
    Serial.println(F("Frequance = 500Hz Val2 = ?"));
    note = 500;
    val2 = 73;
    break;

  case 99: // ComCommand c
    Serial.println(F("Frequance = 600Hz Val2 = ?"));
    note = 600;
    val2 = 83;
    break;

  case 100: // ComCommand d
    Serial.println(F("Frequance = 700Hz Val2 = ?"));
    note = 700;
    val2 = 97;
    break;

  case 101: // ComCommand e
    Serial.println(F("Frequance = 800Hz Val2 = ?"));
    note = 800;
    val2 = 114;
    break;

  case 102: // ComCommand f
    Serial.println(F("Frequance = 900Hz Val2 = ?"));
    note = 900;
    val2 = 134;
    break;

  case 103: // ComCommand g
    Serial.println(F("Frequance = 1000Hz Val2 = 249"));
    note = 1000;
    val2 = 158;
    break;

  case 104: // ComCommand h
    Serial.println(F("Frequance = 1200Hz Val2 = ?"));
    note = 1200;
    val2 = 223;
    break;

  case 105: // ComCommand i
    Serial.println(F("Case = i"));
    break;

  case 106: // ComCommand j
    Serial.println(F("Case = j"));
    break;
  }
}
