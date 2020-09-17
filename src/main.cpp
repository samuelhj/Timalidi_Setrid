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
#include <SPI.h>

// fastar
// Q1-Q4 skilgreindir eftir digital pinnum á Arduino
#define Q1 6
#define Q2 7
#define Q3 8
#define Q4 9
// HIGH er ON og LOW or OFF
#define ON HIGH
#define OFF LOW
// global breytur



// Föll
float voltage_monitor;



// Fall til að fylgjast með Vin
/*
float voltage_monitor(spenna)
{
uint16_t spenna;

return spenna;
}*/



//Uppsetningarfall
void setup()
{

//Digital pinnar 6-9 eru útgangar
  pinMode(6, OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,INPUT_PULLUP); // Inngangur til að athuga stöðu á hleðslutæki


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
//Ef hleðslutæki er að hlaða þá kveikjum við á útgöngum
if(10 == LOW)
{
  digitalWrite(Q1,ON);
  digitalWrite(Q2,ON);
  digitalWrite(Q3,ON);
  digitalWrite(Q4,ON);
}
//Ef hleðslutæki er ekki að hlaða, þá slökkvum við.
if(10 == HIGH)
{
  digitalWrite(Q1,OFF);
  digitalWrite(Q2,OFF);
  digitalWrite(Q3,OFF);
  digitalWrite(Q4,OFF);  
}


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

delay(1000);


}
