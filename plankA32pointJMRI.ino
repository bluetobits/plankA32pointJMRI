//PlankA16 v10.5 32 Points Steve Lomax 07/09/2025 condensed user settings
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Encoder.h>
#include <EEPROM.h>
#include <PCF8575.h>            //https://github.com/RobTillaart/PCF8575
#include <FastLED.h>            //https://github.com/FastLED/FastLED
#include <LiquidCrystal_I2C.h>  //https://github.com/johnrickman/LiquidCrystal_I2C
#include <CMRI.h>               //https://github.com/madleech/ArduinoCMRI/blob/master/CMRI.h
#include <Auto485.h>
//#include <LibPrintf.h>

#define DEBUG 1
#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#define debugln2(x, y) Serial.println(x, y)
#elseplankA32pointJMRI
#define debug(x)
#define debugln(x)
#define debugln2(x, y)
#endif

#define SERVO_FREQ 50  // Analog servos run at ~50 Hz updates

//////////// USER CONFIGURABLE VARIABLES //////////
///////////////////////////////////////////////////

//CHANGE THE VALUES FOR THE i2c ADDRESSES IDENTIFIED ON THE SERIAL MONITOR AT STARTUP. REMEMBER TO SET ADDRESS LINKS
constexpr uint8_t PCF1_ADDRESS = 0X21;  // FIRST EXPANDER
constexpr uint8_t PCF2_ADDRESS = 0X22;  // SECOND EXPANDER
constexpr uint8_t PCA1_ADDRESS = 0X40;  //FIRST SERVO DRIVER
constexpr uint8_t PCA2_ADDRESS = 0X43;  //SECOND SERVO DRIVER
constexpr uint8_t LCD_ADDRESS = 0X27;   //LCD DISPLAY

constexpr uint8_t NO_OF_SERVOS = 32;   // ENTER THE NUMBER OF SERVOS UP TO 16 this can be 16 even if not all implemebted
constexpr uint8_t NO_OF_LEDS = 32;     // ENTER THE NUMBER OF NEO PIXEL LEDs (edit setLeds() function to customise LED logic)
constexpr uint8_t CMRI_ADDRESS = 0;    // CMRI NODE ADDRESS 0
constexpr uint8_t DE_PIN = 5;          // RS485 module
constexpr uint8_t EEPROM_ADDRESS = 0;  // Define EEPROM base address

constexpr uint8_t threeWay = 2;  //the first servo of a 3 way point. if ni 3 way point use 0.

// POINT_PAIRS
// This determines which points are opreated together my one switch.
// the default numerical order infers that each switch operates its own point
// the switch is represented by the number, the point is represented by the position in the list
// a negative number denotes that point thrown / closed position is reversed
// the lowest switch position in a pair or group is the operating switch for the pair or group
// every point must have a switch allocated to it.
// the lowest switch in a group must operate it's own point
// point pairing is disabled when calibrating or disabled in the menu.

constexpr int8_t POINT_PAIRS[] = { 0, 1, 2, 1, 4, 5, 4, 7, 
                                   8, 9, 10, 11, 12, 13, 14, 15,
                                   16, 17, 18, 19, 20, 21, 22, 23, 
                                   24, 25, 26, 27, 28, 29, 30, 31 };
// switches are numbers            0, 1, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15
// points are positions            |  |  -  |  -  |  |  -  |  -  |   -   |   |   |   |
// example point pairs             0, 1, 1, 3,-3, 5, 5, 3, 8, 8, 10,-10, 12, 13, 14, 15
// in this example                    !..!  |  |  !..!  |  !..!  !...!
//                                          !..!........!
// switch 0 -> point 0, switch 1 -> points 1 & 2, switch 2 -> not used, switch 3-> points 3 & 4(reversed) & 7,
// switch 4 not used, switch 5 -> points 5 & 6 , switch 7 not used, switch 8 -> points 8 & 9, ...

//LEDS_MIMIC this is the order of the leds for each point. default is 1 to 1 - led 1 indicates point 1.
// if editing ensure each led is exclusively allocated a point number. by its position in the list.
const uint8_t LEDS_MIMIC[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
                               16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31 };

const uint8_t DATA_PIN = 12;  // neopixels data connect to pin via 50 ohm resistor2
uint8_t onSat = 255;          // Global. Neopixel saturation level (0-255)  255 is pure colour, 122 is pale colour, 0 is white
uint8_t onLev = 80;           // Global. Neopixel brightness level (0-255) 0 is not lit. 255 is max brightness (and current)
uint8_t onHue = 255;          // Global. Neopixel colour ROYGBIV from 0 to 255

constexpr int ENCODER_PUSH = 4;  // pin connection (input pullup switch to gnd)
const uint8_t ENCA_PIN = 3;      // encoder pin A pinmode set by library pin 2 or 3 recommended
const uint8_t ENCB_PIN = 2;      // encoder pin B pinmode set by library pin 2 or 3 recommended

int moveSpeed = 60;    //* initial move speed. increase to make faster. this is adjusted through menu.
int pointPairing = 0;  //* flag if selected. int because memstruct must be the same tyoes

const int MIN_MOVE_SPEED = 1;      // must not be zero
const int MAX_MOVE_SPEED = 100;    // as fast as the servos can travel
const int MID_POINT = 1500;        //for setting servos
const int TOP_PULSE_LEN = 2400;    // *setting the maximum cw servo position(actual = 2500 but not all servos are the same)
const int BOTTOM_PULSE_LEN = 600;  // *setting the minimum ccw servo position


//* these values are overwritten by EEPROM and will only exist until saved from the menu option.
const unsigned long flashSpeed = 400;  // flash rate in ms for LED indicators.
//////////// END OF USER CONGIGURABLE VARIABLES //////////
//unsigned long PollTimeNow;
//unsigned long PollTime = 1500;




int cal = 0;
int localAutomation = 0;  //flag if selected  int because memstruct must be the same types
bool centreServoFlag = 0;
int lastPointMoved = 0;
uint32_t currentStatus = 0;
bool cmriConnected = true;

//int encoderPos = 0;

CRGB leds[NO_OF_LEDS];  //LED neopixel strip
//colour Hues:
const uint8_t Hred = 0;
const uint8_t Hora = 32;
const uint8_t Hyel = 64;
const uint8_t Hgre = 96;
const uint8_t Hcya = 128;
const uint8_t Hblu = 160;
const uint8_t Hpur = 192;
const uint8_t Hpin = 224;
bool moving = 0;
//bool buttonPress = 0;


unsigned long timeNow;
unsigned long flashTimeNow;
unsigned long flashTimeout = 250;  //Global.  Neopixel flash timer


bool flash = 0;  //Global.  Neopixel flash status

Adafruit_PWMServoDriver PCA1 = Adafruit_PWMServoDriver(PCA1_ADDRESS);
Adafruit_PWMServoDriver PCA2 = Adafruit_PWMServoDriver(PCA2_ADDRESS);
PCF8575 PCF1(PCF1_ADDRESS);  // Set the PCF1 I2C address (default is 0x20)
PCF8575 PCF2(PCF2_ADDRESS);  // Set the PCF2 I2C address (default is 0x20)
Encoder encoder(ENCA_PIN, ENCB_PIN);
Auto485 bus(DE_PIN);
LiquidCrystal_I2C lcd(LCD_ADDRESS, 20, 4);  // set the LCD address to 0x27 for a 20 chars and 4 line display
CMRI cmri(CMRI_ADDRESS, 32, 32, bus);       // set CMRI up as SUSIC: Address 0, cardsize: 32-bit, card 0: Input, Card 1: Output. No more cards.


const uint8_t MENU_COUNT = 6;
const char* menuItems[MENU_COUNT] = {
  "Exit", "Calibrate ", "Throw Speed",
  "Point Pairing", "Future", "Centre Servo"
};
uint8_t currentMenuIndex = 0;
long lastEncoderPos = 0;
uint8_t subMenuIndex = 0;
enum SubMenuType {
  CALIBRATION,
  POINT_PAIRING,
  LOCAL_AUTOMATION,
  SET_THROW_SPEED,
  CENTRE_SERVO
};



struct MemStruct {
  int mclosedPos[NO_OF_SERVOS];
  int mthrownPos[NO_OF_SERVOS];
  int mmoveSpeedMem;
  int mpointPairing = 1;     // save point pairing enabled status
  int mlocalAutomation = 0;  //save localAutomation enabled status

  void
  readEEPROM() {
    EEPROM.get(EEPROM_ADDRESS, *this);
  }
  void writeEEPROM() {
    EEPROM.put(EEPROM_ADDRESS, *this);
  }
};

MemStruct memData;


struct Points {
  int closedPos = 700;
  int thrownPos = 2300;
  int curPos = 700;
  bool target = 0;
  bool changed = 0;  // for syncing change status between cmri updates
  void MovePoint(int index) {
    // Determine desired position
    int targetPos = target ? thrownPos : closedPos;
    int delta = targetPos - curPos;  //DTG from current pos.
    if (delta != 0) {
      // Move by at most moveSpeed each call
      int step;                               // distance to move in this call
      if (delta > 0) {                        //clockwise movement
        if (delta < moveSpeed) step = delta;  //if the DTG is less than movespeed set the step to the DTG
        else step = moveSpeed;                // otherwise set step to the movespeed
      } else {
        if (delta > -moveSpeed) step = delta;  // same for anticlockwise.
        else step = -moveSpeed;
      }
      curPos += step;
      moving = true;
      if (index < 16) {
        PCA1.writeMicroseconds(index, curPos);
      } else {
        PCA2.writeMicroseconds(index - 16, curPos);
      }
    }
  }
};

Points point[NO_OF_SERVOS];


void savePointValues() {

  for (int i = 0; i < NO_OF_SERVOS; i++) {
    memData.mthrownPos[i] = point[i].thrownPos;
    memData.mclosedPos[i] = point[i].closedPos;
  }
  memData.mmoveSpeedMem = moveSpeed;
  memData.mpointPairing = pointPairing;
  memData.mlocalAutomation = localAutomation;
  memData.writeEEPROM();
  loadPointValues();
}
void loadPointValues() {
  memData.readEEPROM();
  for (int i = 0; i < NO_OF_SERVOS; i++) {  // transfer memory values to point values
    point[i].thrownPos = memData.mthrownPos[i];
    point[i].closedPos = memData.mclosedPos[i];
    if (point[i].thrownPos < BOTTOM_PULSE_LEN || point[i].thrownPos > TOP_PULSE_LEN) {
      point[i].thrownPos = 1700;
    }
    if (point[i].closedPos < BOTTOM_PULSE_LEN || point[i].closedPos > TOP_PULSE_LEN) {
      point[i].closedPos = 1300;
    }
  }
  moveSpeed = memData.mmoveSpeedMem;
  pointPairing = memData.mpointPairing;
  localAutomation = memData.mlocalAutomation;
  if (pointPairing > 1) pointPairing = 1;
  if (localAutomation > 1) localAutomation = 1;
  if (moveSpeed < 3) moveSpeed = 3;
  if (moveSpeed > 200) moveSpeed = 200;
}

void startPos() {
  int tempspeed = moveSpeed;
  moveSpeed = MID_POINT;
  for (int i = 0; i < NO_OF_SERVOS; i++) {
    // Extract the start position from currentStatus and set as target
    point[i].target = (currentStatus >> i) & 1;
    point[i].MovePoint(i);
    cmri.set_bit(i, point[i].target);
  }
  cmriHandler();
  moveSpeed = tempspeed;
}

void flipMove(int i) {
  point[i].target = !point[i].target;  //flip the target checking for pairs

  point[i].changed = 1;
  point[i].MovePoint(i);

  pointPairs(i);
  lastPointMoved = i;
  lcdPos();                            //writes the position character
  if (cmriConnected) {                 //&& !fromCmr) {
    cmri.set_bit(i, point[i].target);  // Feedback to CMRI
  }
}
void checkThreeWay(int i) {
  if (threeWay > 0) {  //there is a 3 way
    int tempspeed = moveSpeed;
    moveSpeed = 2500;  //fast as possible
    if (i == threeWay) {
      debugln("\n3 way found");
      if (point[i].target == 0 && point[i + 1].target == 1) {
        flipMove(i + 1);
      }
    }
    if (i == threeWay + 1) {
      debugln("\n3 way  + 1 found");
      if (point[i].target == 0 && point[i - 1].target == 1) {
        flipMove(i - 1);
      }
    }
    moveSpeed = tempspeed;
  }
}
void scanButtons() {

  for (int i = 0; i < NO_OF_SERVOS; i++) {
    if (cmriConnected) {
      bool incoming = cmri.get_bit(i);  //saves typing
      if (point[i].changed) {           // out of sync
        if (incoming == point[i].target) {
          point[i].changed = 0;  //jmri now in sync
        }
      }
    }
    if (i < 16) {
      if (!PCF1.read(i)) {
        delay(0);
        debug("\nButton pressed: point ");
        debugln(i);
        checkThreeWay(i);
        flipMove(i);
        while (!PCF1.read(i)) { delay(1); }
      }
    } else {
      if (!PCF2.read(i - 16)) {
        delay(0);
        debugln(i);
        checkThreeWay(i);
        flipMove(i);
        while (!PCF2.read(i - 16)) { delay(1); }
      }
    }


    if (cmriConnected) {
      if (!point[i].changed && cmri.get_bit(i) != point[i].target) {
        debug("\nCMRI clicked  T/O ");
        debugln(i);

        checkThreeWay(i);
        flipMove(i);
      }
    }
  }
}




void pointPairs(int idx) {  //idx = the current switched point
  if (pointPairing) {
    //
    for (int k = idx; k < NO_OF_SERVOS; k++) {  // from current position through to all higher
      if (k != POINT_PAIRS[idx]) {
        if (abs(POINT_PAIRS[k]) == idx) {  // if the value of point Paris  == the current position
          POINT_PAIRS[k] < 0 ? (point[k].target = !point[idx].target) : (point[k].target = point[idx].target);
          point[k].MovePoint(k);
        }
      }
    }
  }
}


void offLeds() {
  for (int i = 0; i < NO_OF_SERVOS; i++) {
    leds[LEDS_MIMIC[i]] = CHSV(Hred, onSat, 0);  // set all to red off
    FastLED.show();
  }
}
void setLeds() {
  flashTimeNow = millis();
  if (flashTimeNow > (flashTimeout)) {         // every 400ms change flash on to flash off or vice versa
    flash = !flash;                            // whatever flash state is (on or off), make it the opposite
    flashTimeout = flashTimeNow + flashSpeed;  //reset the timer
  }
  for (int i = 0; i < NO_OF_SERVOS; i++) {
    // loop through all points
    leds[LEDS_MIMIC[i]] = CHSV(Hred, onSat, onLev);  // set all to red
    if (!point[i].target) {
      leds[LEDS_MIMIC[i]] = CHSV(Hgre, onSat, onLev);  //set closed points to green
    }
    if (pointPairing) {
      if (POINT_PAIRS[i] != i) {
        leds[LEDS_MIMIC[i]] = CHSV(Hora, onSat, onLev);  // set all paired to orange
        if (!point[i].target) {
          leds[LEDS_MIMIC[i]] = CHSV(Hcya, onSat, onLev);  //set closed paired points to cyan
        }
      }
    }

    if (moving) {
      if (!point[i].target) {
        if (point[i].closedPos != point[i].curPos) {
          leds[LEDS_MIMIC[i]] = CHSV(Hblu, onSat, onLev);  //set points not in their correct position to blue (moving points)
        }
      } else {
        if (point[i].thrownPos != point[i].curPos) {
          leds[LEDS_MIMIC[i]] = CHSV(Hblu, onSat, onLev);  //set points not in their correct position to blue (moving points)
        }
      }
    }


    if (flash) {
      if (cal && i == lastPointMoved) {
        leds[LEDS_MIMIC[i]] = CHSV(Hpur, onSat, onLev);  //set points being calibrated to purple
      }
    }

    if (centreServoFlag) {
      if (point[i].thrownPos == MID_POINT && point[i].closedPos == MID_POINT) {
        leds[LEDS_MIMIC[i]] = CHSV(Hyel, onSat, onLev);
      }
    }
  }
  FastLED.show();
}

void lcdGrid() {
  lcd.setCursor(0, 0);
  lcd.print(F("T/O "));
  for (int j = 0; j < 32; j++) {
    if (j >= NO_OF_SERVOS) break;
    if (j == 16) {
      lcd.setCursor(0, 1);
      lcd.print(F("Pos "));
      lcd.setCursor(0, 2);
      lcd.print(F("T/O "));
    }
    lcd.print(j % 10);
  }
  lcd.setCursor(0, 3);
  lcd.print(F("Pos "));
}

void lcdPos() {

  if (currentMenuIndex == 0) {
    lcd.setCursor(4, 1);
    for (int i = 0; i < NO_OF_SERVOS; i++) {
      if (i == 16) lcd.setCursor(4, 3);
      lcd.print(
        (pointPairing && POINT_PAIRS[i] != i) ? (point[i].target ? F("I") : F("\"")) : (point[i].target ? F("|") : F("-")));
    }
  }
}
void lcdPrint() {

  if (cal) {

    lcd.setCursor(0, 1);
    lcd.print(F("Point "));
    lcd.print(lastPointMoved);
    lcd.print(F("  "));
    lcd.setCursor(9, 1);

    if (point[lastPointMoved].target) {
      lcd.print(F("THROWN "));
    } else {
      lcd.print(F("CLOSED "));
    }
    // lcd.setCursor(14, 1);
    lcd.print(point[lastPointMoved].curPos);
    lcd.print(" ");
    // lcd.setCursor(1, 2);
    // lcd.print("Long Push to save");
  }
}
void centreServo() {
  int oldmoveSpeed = moveSpeed;
  moveSpeed = 1500;
  int curPoint = lastPointMoved;
  centreServoFlag = 1;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Select point to"));
  lcd.setCursor(0, 1);
  lcd.print(F("Centre Servo"));
  lcd.setCursor(0, 2);


  while (digitalRead(ENCODER_PUSH) == 1) {
    scanButtons();
    if (curPoint != lastPointMoved) {

      curPoint = lastPointMoved;
      point[curPoint].closedPos = MID_POINT;
      point[curPoint].thrownPos = MID_POINT;
      // point[curPoint].curPos = 0;
      point[curPoint].MovePoint(curPoint);

      lcd.print(curPoint);
      lcd.print(F(" "));

      setLeds();
    }
  }
  while (digitalRead(ENCODER_PUSH) == 0) {}
  delay(100);
  moveSpeed = oldmoveSpeed;
}


//[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[ HANDLESUBMENU ]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]
void handleSubMenu(SubMenuType submenuType, bool hasSaveOption = true) {
  // 1) Determine title, firstOption, and disable Save for Centre Servo
  const char* title;
  const char* firstOption;
  switch (submenuType) {
    case CALIBRATION:
      title = "Calibration";
      firstOption = "Calibrate";
      break;
    case SET_THROW_SPEED:
      title = "Throw Speed";
      firstOption = "Adjust Speed";
      break;
    case POINT_PAIRING:
      title = "Point Pairs";
      firstOption = (pointPairing ? "Disable" : "Enable");
      break;
    case LOCAL_AUTOMATION:
      title = "Local Automation";
      firstOption = (localAutomation ? "Disable" : "Enable");
      break;
    case CENTRE_SERVO:
      title = "Centre Servo";
      firstOption = "Centre Point";
      hasSaveOption = false;
      break;
  }

  // 2) Build the menuStrings array
  const char* menuStrings[3];
  menuStrings[0] = firstOption;
  menuStrings[1] = hasSaveOption ? "Save and Exit" : "Undo and Exit";
  menuStrings[2] = hasSaveOption ? "Undo and Exit" : "";

  int menuItems = hasSaveOption ? 3 : 2;
  int selection = 0;
  int lastSelection = -1;
  long lastEncPos;

  // 3) Initialize encoder
  encoder.write(0);
  lastEncPos = encoder.read() / 4;

  // 4) Initial draw of title + all lines (no arrows yet)
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(title);
  for (int i = 0; i < menuItems; i++) {
    lcd.setCursor(0, i + 1);
    lcd.print("  ");
    lcd.print(menuStrings[i]);
  }

  // 5) Menu loop
  while (true) {
    // a) read encoder and update selection
    long pos = encoder.read() / 4;
    if (pos != lastEncPos) {
      int diff = pos - lastEncPos;
      lastEncPos = pos;

      selection = (selection + diff + menuItems) % menuItems;
    }


    // b) redraw arrows and text if needed
    if (selection != lastSelection) {
      for (int i = 0; i < menuItems; i++) {
        lcd.setCursor(0, i + 1);
        lcd.print(i == selection ? "> " : "  ");
        lcd.print(menuStrings[i]);
      }
      lastSelection = selection;
    }

    // c) handle button press
    if (digitalRead(ENCODER_PUSH) == LOW) {
      while (digitalRead(ENCODER_PUSH) == LOW)
        ;
      delay(100);

      switch (selection) {
        case 0:
          if (submenuType == CALIBRATION) {
            calibrate();
            lastSelection = -1;
          } else if (submenuType == SET_THROW_SPEED) {
            pointMoveSpeed();
            lastSelection = -1;
          } else if (submenuType == POINT_PAIRING) {
            pointPairing = !pointPairing;
            setLeds();
            menuStrings[0] = pointPairing ? "Disable" : "Enable";
            lastSelection = -1;
          } else if (submenuType == LOCAL_AUTOMATION) {
            localAutomation = !localAutomation;
            menuStrings[0] = localAutomation ? "Disable" : "Enable";
            lastSelection = -1;
          } else if (submenuType == CENTRE_SERVO) {
            centreServo();
            lastSelection = -1;
          }
          break;

        case 1:
          if (hasSaveOption) {
            savePointValues();
          } else {
            loadPointValues();
          }
          return;

        case 2:
          // Only valid if hasSaveOption == true
          loadPointValues();
          return;
      }
    }

    delay(50);
  }
}
//[[[[[[[[[[[[[[[[[[[[[[[ POINT MOVE SPEED ]]]]]]]]]]]]]]]]]]]]]]]

void pointMoveSpeed() {
  // 1) Clear & draw static UI
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Adjusting Speed"));
  lcd.setCursor(0, 3);
  lcd.print(F("Press to exit"));
  encoder.write(0);
  int lastSpeedShown = -1;

  while (digitalRead(ENCODER_PUSH) == HIGH) {
    int delta = encoder.read() / 4;
    if (delta != 0) {

      encoder.write(0);
      moveSpeed = constrain(moveSpeed + delta, MIN_MOVE_SPEED, MAX_MOVE_SPEED);
    }
    if (moveSpeed != lastSpeedShown) {
      lastSpeedShown = moveSpeed;
      lcd.setCursor(0, 2);
      lcd.print(F("Speed: "));
      lcd.print(moveSpeed);
      lcd.print(F("    "));
    }

    int tempPointPairing = pointPairing;
    pointPairing = 0;
    scanButtons();

    if (moving) {
      moving = 0;
      for (int i = 0; i < NO_OF_SERVOS; i++) {
        point[i].MovePoint(i);
      }
    }
  }
  while (digitalRead(ENCODER_PUSH) == LOW) {}
  delay(100);
}

//[[[[[[[[[[[[[[[[[[[[[[[[[[[[ CALIBRATE ]]]]]]]]]]]]]]]]]]]]]]]]]]]]
void calibrate() {
  cal = 1;  // set cal flag
  // Backup current state
  int originalPointPairing = pointPairing;
  pointPairing = false;

  int originalMoveSpeed = moveSpeed;
  moveSpeed = 2500;

  // Initialize encoder
  encoder.write(0);
  lastEncoderPos = encoder.read() / 4;

  lcd.clear();

  // Draw static elements once
  lcd.setCursor(0, 0);
  lcd.print(F("Adjusting Point"));
  lcd.setCursor(0, 3);
  lcd.print(F("Turn/Change/Press  "));  // 🔹 Bug 2 fix


  int lastPosShown = -1;
  int lastPoint = -1;
  int lastTarget = -1;

  // Calibration loop
  while (digitalRead(ENCODER_PUSH) == HIGH) {
    scanButtons();  // Updates lastPointMoved and toggles its target
    setLeds();

    int& targetPos = (point[lastPointMoved].target == 0)
                       ? point[lastPointMoved].closedPos
                       : point[lastPointMoved].thrownPos;

    // Encoder adjustment
    int pos = encoder.read();
    if (pos != lastEncoderPos) {
      int diff = pos - lastEncoderPos;
      lastEncoderPos = pos;

      targetPos += diff;
      targetPos = constrain(targetPos, BOTTOM_PULSE_LEN, TOP_PULSE_LEN);
      point[lastPointMoved].curPos = targetPos;
      //point[lastPointMoved].MovePoint(lastPointMoved);  // ✅ this line is essential
      // if (lastPointMoved < 16) {
      PCA1.writeMicroseconds(lastPointMoved, targetPos);
      // } else {
      //   PCA2.writeMicroseconds(lastPointMoved - 16, targetPos);
      // }
      setLeds();
    }

    // Only update display if values change
    if (lastPoint != lastPointMoved || lastTarget != point[lastPointMoved].target || lastPosShown != point[lastPointMoved].curPos) {
      lastPoint = lastPointMoved;
      lastTarget = point[lastPointMoved].target;
      lastPosShown = point[lastPointMoved].curPos;

      lcd.setCursor(0, 1);
      lcd.print(F("Point: "));
      if (lastPointMoved < 10) lcd.print(F("0"));
      lcd.print(lastPointMoved);
      lcd.print(F(" "));
      lcd.print(lastTarget == 0 ? F("CLOSED") : F("THROWN"));
      lcd.print(F("   "));  // clear residual text

      lcd.setCursor(0, 2);
      lcd.print(F("Pos: "));
      lcd.print(lastPosShown);
      lcd.print(F("  "));
    }

    delay(50);  // debounce and flicker protection
  }


  while (digitalRead(ENCODER_PUSH) == LOW) {}  // Wait for button release
  delay(100);
  encoder.write(0);
  lcd.clear();  //
  // Redraw calibration menu
  lcd.setCursor(0, 0);


  pointPairing = originalPointPairing;
  moveSpeed = originalMoveSpeed;
  cal = 0;  //
}

//[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]
//[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[ MAIN MENU ]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]
//[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]
void lcdControlMenu() {

  if (digitalRead(ENCODER_PUSH) == HIGH) return;

  while (digitalRead(ENCODER_PUSH) == LOW) {}  // Wait for release

  currentMenuIndex = 0;
  encoder.write(currentMenuIndex * 4);
  lastEncoderPos = encoder.read() / 4;
  int lastMenuIndex = -1;

  displayMenu(currentMenuIndex);  // Initial draw

  while (true) {
    long pos = encoder.read() / 4;

    if (pos != lastEncoderPos) {
      int diff = pos - lastEncoderPos;
      lastEncoderPos = pos;
      currentMenuIndex = constrain((currentMenuIndex + diff), 0, MENU_COUNT - 1);
    }

    if (currentMenuIndex != lastMenuIndex) {
      displayMenu(currentMenuIndex);
      lastMenuIndex = currentMenuIndex;
    }

    if (digitalRead(ENCODER_PUSH) == LOW) {
      while (digitalRead(ENCODER_PUSH) == LOW)
        ;  // Wait for release
      lcd.clear();
      switch (currentMenuIndex) {
        case 0:

          lcdGrid();
          lcdPos();
          return;

        case 1:
          handleSubMenu(CALIBRATION);
          break;

        case 2:
          handleSubMenu(SET_THROW_SPEED);
          break;

        case 3:
          handleSubMenu(POINT_PAIRING);
          break;

        case 4:
          handleSubMenu(LOCAL_AUTOMATION);
          break;

        case 5:
          handleSubMenu(CENTRE_SERVO, false);  // No Save option
          break;
      }

      // Restore menu index after submenu
      encoder.write(currentMenuIndex * 4);
      lastEncoderPos = encoder.read() / 4;
      lastMenuIndex = -1;  // Force redraw
    }

    delay(50);  // debounce
  }
}

void displayMenu(int selectedIndex) {
  lcd.clear();


  int topIndex = selectedIndex < 2 ? 0 : selectedIndex > MENU_COUNT - 3 ? MENU_COUNT - 4
                                                                        : selectedIndex - 1;

  for (int i = 0; i < 4; i++) {
    int menuIndex = topIndex + i;
    lcd.setCursor(0, i);
    lcd.print((menuIndex == selectedIndex) ? "->" : "  ");
    lcd.print(menuItems[menuIndex]);
  }
}

void centreServoPos(int pointNo) {


  int tempMoveSpeed = moveSpeed;
  moveSpeed = MID_POINT;

  point[pointNo].thrownPos = MID_POINT;
  point[pointNo].closedPos = MID_POINT;
  point[pointNo].curPos = 0;
  point[pointNo].target = 0;
  point[pointNo].MovePoint(pointNo);
  point[pointNo].target = 1;
  point[pointNo].MovePoint(pointNo);

  moveSpeed = tempMoveSpeed;
}
void cmriHandler() {

  bool cmriCxOld = cmriConnected;
  //Serial.print(cmriConnected);
  cmriConnected = (cmri.process() != NULL);
  //Serial.println(cmriConnected);
  if (cmriCxOld != cmriConnected) {
    debug("\n");
    if (!cmriConnected) debug("NO ");
    debugln("CMRI");
    
  }
}


//[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]
//[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[ SETUP ]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]
//[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]
void setup() {
  Wire.begin();
  Serial.begin(19200);
  PCF1.begin();
  //PCF2.begin();

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NO_OF_SERVOS);

  Serial.println("plankA32 v10.5 07/09/2025  ");
  offLeds();
  int nDevices = 0;
  Serial.println("Scanning I2C ");

  for (byte address = 1; address < 127; ++address) {
    // The i2c_scanner uses the return value of
    // the Wire.endTransmission to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print(F("I2C device at 0x"));
      if (address < 16) {
        Serial.print(F("0"));
      }
      Serial.print(address, HEX);
      Serial.print("  !");
      if (address < 20) {
        Serial.println(F(" Arduino or other MCU?"));
      } else if (address < 0x27) Serial.println(F(" I/O expander PSF ?"));
      else if (address < 0x30) Serial.println(F(" LCD Dispay"));
      else if (address < 0x50) Serial.println(F(" ServoDriver PCA9685"));
      else if (address < 0x80) Serial.println(F(" Unknown device"));
      ++nDevices;
    } else if (error == 4) {
      Serial.print(F("error at 0x"));
      if (address < 16) {
        Serial.print(F("0"));
      }
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println(F("No I2C devices found\n"));
  }
  lcd.init();  // initialize the lcd
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(F("Point control v10.5"));
  lcd.setCursor(0, 1);
  lcd.print(F("Steve Lomax 09/25"));
  delay(1000);

  encoder.write(0);

  pinMode(ENCODER_PUSH, INPUT_PULLUP);
  loadPointValues();

  PCA1.begin();
  PCA1.setPWMFreq(50);
  PCA1.setOscillatorFrequency(25000000);
  PCA2.begin();
  PCA2.setPWMFreq(50);
  PCA2.setOscillatorFrequency(25000000);



  PCF1.write16(0Xffff);
  debug("PCF1 = ");
  debugln2(PCF1.read16(), BIN);

  PCF2.write16(0Xffff);
  debug("PCF2 = ");
  debugln2(PCF2.read16(), BIN);


  lcd.setCursor(3, 3);
  lcd.print(F("Loading..."));
  if (digitalRead(ENCODER_PUSH) == 0) {
    lcd.setCursor(0, 2);
    lcd.print(F("Hold = Centre Servo"));
    delay(2000);
  }
  if (digitalRead(ENCODER_PUSH) == 0) {
    lcd.setCursor(0, 3);
    lcd.print(F("Reposition points"));

    for (int i = 0; i < NO_OF_SERVOS; i++) {
      centreServoPos(i);
      centreServoFlag = 1;
    }
  }
  Serial.end();
  delay(1000);
  bus.begin(19200);


  startPos();

  lcd.clear();
  lcdGrid();
  lcdPos();
}


void loop() {

  scanButtons();

  if (moving) {

    moving = 0;
    for (int i = 0; i < NO_OF_SERVOS; i++) {
      point[i].MovePoint(i);
    }
  } else {
    cmriHandler();
    lcdPos();
    lcdControlMenu();
  }
  setLeds();
}
