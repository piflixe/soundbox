/***************************************************
DFPlayer - A Mini MP3 Player For Arduino
 <https://www.dfrobot.com/index.php?route=product/product&product_id=1121>

 ***************************************************
 This example shows the basic function of library for DFPlayer.

 Created 2016-12-07
 By [Angelo qiao](Angelo.qiao@dfrobot.com)

 GNU Lesser General Public License.
 See <http://www.gnu.org/licenses/> for details.
 All above must be included in any redistribution
 ****************************************************/

/***********Notice and Trouble shooting***************
 1.Connection and Diagram can be found here
 <https://www.dfrobot.com/wiki/index.php/DFPlayer_Mini_SKU:DFR0299#Connection_Diagram>
 2.This code is tested on Arduino Uno, Leonardo, Mega boards.
 ****************************************************/

#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include <OneButton.h>
#include <SPI.h>
#include <MFRC522.h>


SoftwareSerial mySoftwareSerial(5, 6); // RX, TX
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);

const int PIN_VOLUME = A0;
const int PIN_NEXT = A2;
const int PIN_LED = 4;
const int PIN_VCC = A3;
const int UPDATE_INTERVAL = 100; // update interval in ms
const int BLINK_INTERVAL = 800; // LED blink interval in ms
const int BATTERY_INTERVAL = 5000; // interval for measuring battery voltage
const int SS_PIN = 10;
const int RST_PIN = 9;

// DFPlayer States
const int DFPLAYER_PLAYING = 1;
const int DFPLAYER_PAUSED = 2;

bool ledPinState = false;
int volume = 10;
int vcc = 0;

// unsigned long id = 0;


OneButton button_next = OneButton(
  PIN_NEXT, // Input pin for the button
  true, // Button is active LOW
  true // Enable internal pull-up resistor
);

// RFID Reader
MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance

void setup()
{
  mySoftwareSerial.begin(9600);
  Serial.begin(115200);

  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true);
  }
  Serial.println(F("DFPlayer Mini online."));
  pinMode(PIN_VOLUME, INPUT);
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED,ledPinState);
  pinMode(PIN_VCC,INPUT);

  button_next.attachClick(nextSong);
  button_next.attachLongPressStart(pause);

  myDFPlayer.volume(volume);  //Set volume value. From 0 to 30
  myDFPlayer.play(1);  //Play the first mp3

  SPI.begin();
  mfrc522.PCD_Init();
 	// mfrc522.PCD_DumpVersionToSerial();	// Show details of PCD - MFRC522 Card Reader details
  Serial.println("ready to read ID...");
}

void loop()
{
  static unsigned long timer = millis();

  if (millis() - timer > UPDATE_INTERVAL) {
    timer = millis();

    // read poti and set volume in DFPlayer
    volume = map(analogRead(PIN_VOLUME), 0, 1024, 0, 30);
    myDFPlayer.volume(volume);
    // Serial.println(volume);

    // myDFPlayer.next();  //Play next mp3 every 3 second.
  }

  static unsigned long blinkTimer = millis();
  if (millis() - blinkTimer > BLINK_INTERVAL) {
    blinkTimer = millis();
    // blink LED when playing
    if(myDFPlayer.readState() == 1){
      digitalWrite(PIN_LED,ledPinState);
      ledPinState = !ledPinState;
    }
    // Serial.println(myDFPlayer.readState()); //read mp3 state

  }

  static unsigned long batteryTimer = millis();
  if (millis() - batteryTimer > BATTERY_INTERVAL) {
    analogReference(INTERNAL); 
    batteryTimer = millis();
    for(int i=0;i<10;i++){ // do a few samples after change of analog reference
      analogRead(PIN_VCC);
      delay(2);
    }
    vcc = 0;
    for(int i=0;i<5;i++){
      vcc = vcc + map(analogRead(PIN_VCC),0,1023,0,6060);
    }
    vcc = vcc / 5;
    // Serial.println(vcc);
    analogReference(DEFAULT); // set analog reference back to vcc and collect a few samples 
    for(int i=0;i<10;i++){ // do a few samples after change of analog reference
      analogRead(PIN_VCC);
      delay(2);
    }
  }

  if (myDFPlayer.available()) {
    // printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }

  button_next.tick();

  // Look for new cards
if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    // Combine ID bytes into a single 32-bit integer
    uint64_t idInt = 0;
    Serial.print("Card ID: ");
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? "0" : "");
      Serial.print(mfrc522.uid.uidByte[i], HEX);
      idInt <<= 8;
      idInt |= mfrc522.uid.uidByte[i];
    }
    Serial.println();
    // Serial.println((uint32_t)idInt,HEX); // since Serial.println only supports uint32 variables, it is casted on the fly, but the real UID can be larger and sometimes only changes 
    mfrc522.PICC_HaltA(); // Stop reading
    if(idInt == 0x04BCF6C2613E80){
    Serial.println("YES!");
  }
  }
}

void printDetail(uint8_t type, int value){
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}


static void nextSong() {
  Serial.println("normal Click!");
  if(myDFPlayer.readState() == DFPLAYER_PLAYING){
    myDFPlayer.next();
  }
  else {
    myDFPlayer.play();
  }
}

static void pause() {
  Serial.println("long click!");
  myDFPlayer.pause();
  digitalWrite(PIN_LED,LOW);
}
