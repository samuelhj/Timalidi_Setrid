# Timalidi_Setrid

Tímaliði sem stýrist af sólargangingum hvern mánuð. 
Einnig getur rásin notað tvo interrupt pinna sem eru forritanlegir eða notað ALARM frá RTC rás í gegnum interrupt.

Fjórir forritanlegir MOSFET útgangar.


Kóðinn (src/main.c) er gefinn út undir BSD 3 leyfi. Hver sem er má nota kóðann í hvaða tilgangi sem er, þarf bara að vísa í uppruna hans.
Öll kóðasöfn sem eru notuð í þessu verkefni helgast af þeim leyfum sem þau eru gefin út undir og þarf viðkomandi að hafa það í huga.


RTC Rás er Chronodot v2.0 http://docs.macetech.com/doku.php/chronodot_v2.0

Eagle skrár: https://ulfraf.space/Projects/Ulfraf/timalidi_setrid.tar.gz

Höfundur er Samúel Þór Hjaltalín

Þetta verkefni er langt frá því að vera klárað! Senda má fyrirspurnir á samuel@ulfr.net 

Library sem þetta forrit notar.

Fyrir RTC:

https://github.com/JChristensen/DS3232RTC

https://github.com/mizraith/RTClib

Önnur kóðasöfn eru standard AVR/Arduino.

SPI.h

avr/sleep.h

Wire.h

Arduino.h


Það sem virkar:
RTC rás getur vakið upp AVR í gegnum Alarm1 via INTERRUPT1

AVR vaknar við utanaðkomandi snertu í gegnum INTERRUPT0

Það sem er í vinnslu:

Dagatal vantar.

Draga úr straumnotkun.

