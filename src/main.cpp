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
  Janúar    11:00 - 16:00 5 klst on
  Febrúar   10:00 - 18:00 8 klst on
  Mars      08:00 - 19:30 11,5klst on
  Apríl     06:00 - 21:00 15 klst on
  Maí       04:00 - 22:30
  Júní      Alltaf kveikt
  Júlí      03:30 - 23:30   Alltaf kveikt
  Ágúst     05:30 - 21:30   Alltaf kveikt?
  September 07:00 - 20:00
  Október   08:30 - 18:00
  Nóvember  10:00 - 16:30
  Desember  11:30 - 15:30

  Listi yfir þekkt vandamál og viðbætur sem þarf að bæta við

  Fá dagatal til að virka
  Status díóður þurfa betri virkni.
  Alarm til að virkja rás þegar það er hádegi.
  Skoða hvaða möguleika hægt er að draga úr straumnotkun enn frekar.

*/


#include <Wire.h>
#include <Arduino.h>
#include <RTClib.h> // Lib fyrir real time clock
//#include <RTC_DS3231.h> // lib fyrir real time clock
//#include <DS3231.h> //
#include <DS3232RTC.h> // https://github.com/JChristensen/DS3232RTC
#include <SPI.h> // SPI samskipti
#include <avr/sleep.h> // svefn
//#include <avr/interrupt.h>
//#include <avr/power.h>
//#include <avr/io.h>

//RTC_DS3231 RTC;

// fastar
// Q1-Q4 skilgreindir eftir digital pinnum á Arduino
// Þetta eru MOSFET rásir sem stýra jaðarbúnaði.

#define Q1 6 // Q1 er stýrt af pinna 6
#define Q2 7 // Q2 er stýrt af pinna 7
#define Q3 8 // Q3 er stýrt af pinna 8
#define Q4 9 // Q4  er stýrt af pinna 9

// Skilgreiningar fyrir Status LED
#define STATUS_LED_GREEN 4 // Pinni 4
#define STATUS_LED_RED 5 // Pinni 5

// pinnar fyrir external interrupt
#define INTERRUPT0 2 // Pinni 2
#define INTERRUPT1 3 // Pinni 3

// HIGH er ON og LOW or OFF
#define ON HIGH
#define OFF LOW

// global breytur
uint8_t manudur = 0; // gildi fyrir mánuðinn. 1 - 12
uint8_t erdagur = 0; // Er dagur eða ekki? Notað til að ákveða hvort það sé kveikt
uint16_t keyrslutimi = 6000; // Breyta fyrir hve lengi við keyrum
uint16_t hours; // Breyta fyrir klukkustundir
uint16_t minutes; // Breyta fyrir Mínútur
uint16_t seconds; // Breyta fyrir sekúndur
uint16_t timer0 = 0; // 16 bita teljari
uint16_t timer1 = 0; // 16 bita teljari
uint8_t timer2 = 0; // 8 bita teljari
uint8_t q_onoroff = 0; // Breyta sem geymir hvort við kveikjum á útgöngum eða ekki
// Föll

void wakeUp(); // Fall til að vekja upp AVR
int er_dagur(); // Fall til að athuga hvort það sé dagur
void er_hadegi(); // Fall til að athuga hvort það sé hádegi
void kveikt(); // Kveikja á útgöngum og stöðu LED
void slokkva(); // fall til að slökkva á útgöngum og stöðu LED
void writeControlByte(); // Fall til að stilla RTC Rás.

// Hér byrja undirlykkjur

// Fall til að vekja
void wakeUp()
{
  //Serial.println("Interrupt fired!");
  sleep_disable(); // Förum úr svefni
  detachInterrupt(0); // Aftengjum interrupt0 svo við lendum ekki í lúppu
  detachInterrupt(1); // Aftengjum interrupt1
}

//Fall til að athuga hvort það sé hádegi. Til að byrja með ætlum við bara að kveikja á útgöngum
// Þegar það er hádegi, síðar meir verður breytt yfir í dagatal þegar sólarsellan er kominn
// á sinn stað
void er_hadegi()
{
  // ef kl er 12
  if((hours == 12) && (minutes == 0))
  {
    digitalWrite(STATUS_LED_GREEN,ON);
  }

}

//Fall til að athuga hvort það sé dagur eða nótt
int er_dagur()
{
  uint8_t status = 0; // breyta fyrir útkomu, er dagur eða nótt?

  // Þarf að vinna í þessu. Spurning með hvernig sé best að fá mánuðinn frá RTC?
  return status; // Við skilum til baka ef það er dagur eða nótt
}

// Fall sem kveikir á útgöngum og status díóðum
void kveikt()
{
  digitalWrite(Q1,ON);
  digitalWrite(Q2,ON);
  digitalWrite(Q3,ON);
  digitalWrite(Q4,ON);
  digitalWrite(STATUS_LED_GREEN,HIGH);
  delay(100);
  digitalWrite(STATUS_LED_GREEN,LOW);
  delay(1000);
}

void slokkva()
{
  digitalWrite(Q1,OFF);
  digitalWrite(Q2,OFF);
  digitalWrite(Q3,OFF);
  digitalWrite(Q4,OFF);
  digitalWrite(13,LOW);
}

void writeControlByte(byte control, bool which) {  // Set DS3121 RTC control bytes
   // Write the selected control byte.
   // which=false -> 0x0e, true->0x0f.
   Wire.beginTransmission(0x68);
   if (which) {
      Wire.write(0x0f);
   } else {
      Wire.write(0x0e);
   }
   Wire.write(control);
   Wire.endTransmission();
}


//Uppsetningarfall
void setup()
{

//Digital pinnar 6-9 eru útgangar
  pinMode(STATUS_LED_GREEN,OUTPUT); // Status LED
  pinMode(STATUS_LED_RED,OUTPUT); // Status LED
  pinMode(Q1,OUTPUT); // Útgangur 1
  pinMode(Q2,OUTPUT); // Útgangur 2
  pinMode(Q3,OUTPUT); // Útgangur 3
  pinMode(Q4,OUTPUT); // Útgangur 4
  pinMode(INTERRUPT0,INPUT_PULLUP); // Interrupt pinni til að athuga stöðu á hleðslutæki
  pinMode(INTERRUPT1,INPUT_PULLUP); // Interrupt pinni 2, notað til að ræsa AVR til að ath tímann

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


  // Alarm 1
  // Hreinsum gildi
  RTC.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);
  RTC.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);
  RTC.alarm(ALARM_1);
  RTC.alarm(ALARM_2);
  RTC.alarmInterrupt(ALARM_1, false);
  RTC.alarmInterrupt(ALARM_2, false);
  RTC.squareWave(SQWAVE_NONE);

  // set the RTC time and date to the compile time
  //RTC.set(compileTime());

  //RTC.setAlarm(alarmType, seconds, minutes, hours, dayOrDate);
  RTC.setAlarm(ALM1_MATCH_MINUTES, 0, 1, 0, 0); // 5 mínútna fresti fyrir test
  // clear the alarm flag
  RTC.alarm(ALARM_1);
  // enable interrupt output for Alarm 1
   RTC.alarmInterrupt(ALARM_1, true);

} // void setup() endar


//Aðalfall
void loop()
{

  timer1 = millis(); // timer1 geymir keyrslutíma AVR gaursins.

  int relay = digitalRead(INTERRUPT0); // breyta fyrir snertu frá Victron hleðslutækinu
  int relay2 = digitalRead(INTERRUPT1); // Breyta fyrir SQW/INT frá RTC rás.

  // RTC rás vekur upp AVR.
  // AVR athugar fyrst í sinn bara hvort það sé að nálgast hádegi.
  // Ef það er hádegi, þá keyrum við upp rásina og ræsum myndavélar og router.
  if(relay2 == LOW)
  {
    //Hér lesum við tímann og athugum hvað kl er.
    q_onoroff = 1; // test
  }

  // Ef Hleðslutækið er í gangi þá kveikjum við ALLTAF á myndavélunum.
  if(relay == LOW)
  {
    q_onoroff = 1;
  }
  // ef það má kveikja á útgöngum
  if(q_onoroff == 1)
  {
    digitalWrite(Q1,ON); // Ræsum Q1 fyrir router
    digitalWrite(Q2,ON); // Ræsum Q2 fyrir myndavél A
    digitalWrite(Q3,ON); // Ræsum Q3 fyrir Myndavél B

    // ef við erum búin að kveikja á útgöngum þá könnum við
    if(timer1 - timer0 >= keyrslutimi)
    {
        //q_onoroff = 0; //  Segjum forriti að það megi slökkva
        timer0 = timer1;
        timer2 = timer2+1;
        // ef við höfum farið 10 sinnum í gegnum teljarann timer2
        // sem eru 10 mínútur. Það ætti að gefa Myndavél og router tíma til að ræsa sig upp
        // taka mynd, og senda frá sér. Gæti þurft að stilla þetta eitthvað.
        if(timer2 > 10)
        {
          q_onoroff = 0; // Þá slökkvum við.
          timer2 = 0; // endursetjum teljarann

        }
    } // Fall fyrir teljara endar

      // Debugging

      // send request to receive data starting at register 0
      Wire.beginTransmission(0x68); // 0x68 is DS3231 device address
      Wire.write((byte)0); // start at register 0
      Wire.endTransmission();
      Wire.requestFrom(0x68, 3); // request three bytes (seconds, minutes, hours)

      while(Wire.available())
      {
        // blikkum status leddu tvisvar snöggt
        for(int a=0;a<3;a++)
        {
          digitalWrite(STATUS_LED_GREEN,ON); // kveikjum á status leddu
          delay(50);
          digitalWrite(STATUS_LED_GREEN,OFF);// slökkvum á status leddu
          delay(500);
        }


        seconds = Wire.read(); // get seconds
        minutes = Wire.read(); // get minutes
        hours = Wire.read();   // get hours

        seconds = (((seconds & 0b11110000)>>4)*10 + (seconds & 0b00001111)); // convert BCD to decimal
        minutes = (((minutes & 0b11110000)>>4)*10 + (minutes & 0b00001111)); // convert BCD to decimal
        hours = (((hours & 0b00100000)>>5)*20 + ((hours & 0b00010000)>>4)*10 + (hours & 0b00001111)); // convert BCD to decimal (assume 24 hour mode)

        Serial.print(hours); Serial.print(":"); Serial.print(minutes); Serial.print(":"); Serial.println(seconds);
      }// RTC lestur endar

    }// Útgangar ON fall endar

    // Ef það á að vera slökkt á öllum útgöngum
  if(q_onoroff == 0)
  {
    slokkva();
    digitalWrite(STATUS_LED_RED,ON);
    delay(100);
    digitalWrite(STATUS_LED_RED,OFF);

    // og förum aftur að sofa
    set_sleep_mode (SLEEP_MODE_PWR_DOWN);
    sleep_enable();

    noInterrupts();
    // will be called when pin D2 goes low
    attachInterrupt (0, wakeUp, LOW); // INTERRUPT0
    attachInterrupt(1, wakeUp, LOW);  // INTERRUPT1
    EIFR = bit (INTF0);  // clear flag for interrupt 0
    EIFR = bit (INTF1); // Hreinsum flagg fyrir INTERRUPT1

    // We are guaranteed that the sleep_cpu call will be done
    // as the processor executes the next instruction after
    // interrupts are turned on.
    interrupts ();  // one cycle
    sleep_cpu ();   // one cycle
  } // Fall fyrir svefn og OFF á útgöngum endar


} // void loop() endar
