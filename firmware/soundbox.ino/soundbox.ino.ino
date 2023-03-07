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
#include "Button2.h"
#define LONGCLICK_MS 1000

SoftwareSerial mySoftwareSerial(10, 11); // RX, TX
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);

Button2 button_next;

const int PIN_VOLUME = A0;
const int PIN_NEXT = A2;
const int PIN_LED = 4;
const int UPDATE_INTERVAL = 100; // update interval in ms
const int BLINK_INTERVAL = 800; // LED blink interval in ms

// DFPlayer States
const int DFPLAYER_PLAYING = 1;
const int DFPLAYER_PAUSED = 2;

bool ledPinState = false;
int volume = 10;


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

  button_next.begin(PIN_NEXT,INPUT_PULLUP,true);
  button_next.setClickHandler(nextSong);
  button_next.setLongClickHandler(pause);

  myDFPlayer.volume(volume);  //Set volume value. From 0 to 30
  myDFPlayer.play(1);  //Play the first mp3
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
    Serial.println(myDFPlayer.readState()); //read mp3 state
  }

  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }

  button_next.loop();
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

void nextSong(Button2& b) {
  Serial.println("normal Click!");
  if(myDFPlayer.readState() == DFPLAYER_PLAYING){
    myDFPlayer.next();
  }
  else {
    myDFPlayer.play();
  }
}

void pause(Button2& b) {
  Serial.println("long click!");
  myDFPlayer.pause();
  digitalWrite(PIN_LED,LOW);
}