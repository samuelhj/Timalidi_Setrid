/*
  Forrit fyrir tímaliða sem stýrir 4 útgöngum.

  Hlutverk: RTC rás heldur tímann og AVR sér um að "muna" hvenær sólarupprás
  hefst og hvenær sólsetur er. Kveikt er á útgöngum við Sólarupprás og slökkt
  c.a. 1 klst eftir sólsetur.
  Að auki eru útgangar alltaf virkir ef hleðslutæki fyrir rafhlöður er í gangi.
  Það er skoðað með að mæla relay á Victron Phoenix hleðslustýringu.

RTC rásin er Chronodot v2.0 http://docs.macetech.com/doku.php/chronodot_v2.0

Kóðasöfn sem þarf:
  https://github.com/mizraith/RTClib
  https://github.com/jarzebski/Arduino-DS3231



  Listi yfir sólarupprás og sólsetur og hvenær útgangar eru virkir.
  Janúar    11:00 - 16:00
  Febrúar   10:00 - 18:00
  Mars      08:00 - 19:30
  Apríl     06:00 - 21:00
  Maí       04:00 - 22:30
  Júní      Alltaf kveikt
  Júlí      03:30 - 23:30   Alltaf kveikt
  Ágúst     05:30 - 21:30   Alltaf kveikt?
  September 07:00 - 20:00
  Október   08:30 - 18:00
  Nóvember  10:00 - 16:30
  Desember  11:30 - 15:30
*/


#include <Wire.h>
#include <Arduino.h>
#include <RTClib.h> // Lib fyrir real time clock
#include <SPI.h> // SPI samskipti
#include <avr/sleep.h> // svefn

// fastar
// Q1-Q4 skilgreindir eftir digital pinnum á Arduino
// Þetta eru MOSFET rásir sem stýra jaðarbúnaði.

#define Q1 6
#define Q2 7
#define Q3 8
#define Q4 9

// HIGH er ON og LOW or OFF
#define ON HIGH
#define OFF LOW

// global breytur
uint8_t manudur = 0; // gildi fyrir mánuðinn. 1 - 12
uint8_t erdagur = 1; // Er dagur eða ekki? Notað til að ákveða hvort það sé kveikt

// Föll

void wakeUp();
int er_dagur();

//Fall til að athuga hvort það sé dagur eða nótt
int er_dagur()
{
  uint8_t status = 0; // breyta fyrir útkomu, er dagur eða nótt?

  // Þarf að vinna í þessu. Spurning með hvernig sé best að fá mánuðinn frá RTC?

  return status; // Við skilum til baka ef það er dagur eða nótt
}
// Fall til að vekja
void wakeUp()
{
  //Serial.println("Interrupt fired!");
  sleep_disable(); // Förum úr svefni 
  detachInterrupt(0); // Aftengjum interrupt svo við lendum ekki í lúppu
}
//Uppsetningarfall
void setup()
{

//Digital pinnar 6-9 eru útgangar
  pinMode(6, OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT); // Status díóða.
  pinMode(2,INPUT_PULLUP); // Interrupt pinni til að athuga stöðu á hleðslutæki
  pinMode(13,OUTPUT); // fyrir Status LED2
  digitalWrite(13,HIGH);

  Wire.begin();
  Serial.begin(9600);

  // clear /EOSC bit
  // Sometimes necessary to ensure that the clock
  // keeps running on just battery power. Once set,
  // it shouldn't need to be reset but it's a good
  // idea to make sure.
  Wire.beginTransmission(0x68); // address DS3231
  Wire.write(0x0E); // select register
  Wire.write(0b00011100); // write register bitmap, bit 7 is /EOSC
  Wire.endTransmission();

}

//Aðalfall
void loop()
{

  int relay = digitalRead(2); // breyta fyrir snertu frá Victron hleðslutækinu
  // Ef Hleðslutækið er í gangi þá kveikjum við ALLTAF á myndavélunum.
  // sem og ef það er dagur
  if((relay == LOW) && (erdagur == 1))
  {
    digitalWrite(Q1,ON);
    digitalWrite(Q2,ON);
    digitalWrite(Q3,ON);
    digitalWrite(Q4,ON);
    digitalWrite(13,HIGH);
    delay(500);
    digitalWrite(13,LOW);
    delay(500);
  }
  // Ef hleðslutækið slekkur á sér þá slökkvum við á myndavélum sem og
  // ef það er nótt.
  if((relay == HIGH) && (erdagur == 0))
  {
    digitalWrite(Q1,OFF);
    digitalWrite(Q2,OFF);
    digitalWrite(Q3,OFF);
    digitalWrite(Q4,OFF);
    digitalWrite(13,LOW);
  }
  // og förum aftur að sofa
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  noInterrupts();
  // will be called when pin D2 goes low
  attachInterrupt (0, wakeUp, LOW);
  EIFR = bit (INTF0);  // clear flag for interrupt 0

  // We are guaranteed that the sleep_cpu call will be done
  // as the processor executes the next instruction after
  // interrupts are turned on.
  interrupts ();  // one cycle
  sleep_cpu ();   // one cycle





// Debugging

  // send request to receive data starting at register 0
Wire.beginTransmission(0x68); // 0x68 is DS3231 device address
Wire.write((byte)0); // start at register 0
Wire.endTransmission();
Wire.requestFrom(0x68, 3); // request three bytes (seconds, minutes, hours)

while(Wire.available())
{
  int seconds = Wire.read(); // get seconds
  int minutes = Wire.read(); // get minutes
  int hours = Wire.read();   // get hours

  seconds = (((seconds & 0b11110000)>>4)*10 + (seconds & 0b00001111)); // convert BCD to decimal
  minutes = (((minutes & 0b11110000)>>4)*10 + (minutes & 0b00001111)); // convert BCD to decimal
  hours = (((hours & 0b00100000)>>5)*20 + ((hours & 0b00010000)>>4)*10 + (hours & 0b00001111)); // convert BCD to decimal (assume 24 hour mode)

  Serial.print(hours); Serial.print(":"); Serial.print(minutes); Serial.print(":"); Serial.println(seconds);
}

delay(100);

}
