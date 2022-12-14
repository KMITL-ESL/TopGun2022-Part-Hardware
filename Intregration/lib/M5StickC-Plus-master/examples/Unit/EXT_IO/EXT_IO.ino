#include <M5StickCPlus.h>
#include "PCA9554.h"  // Load the PCA9554 Library

PCA9554 ioCon1(0x27);  // Create an object at this address

uint8_t res;
void setup() {
    M5.begin();
    Wire.begin();
    // M5.Lcd.clear(BLACK);
    M5.Lcd.setRotation(3);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(YELLOW);
    M5.Lcd.setCursor(32, 2);
    M5.Lcd.print("UNIT_IO EXAMPLE\n");

    ioCon1.twiWrite(32, 33);
    delay(10);
    res = 1;
    ioCon1.twiRead(res);
    Serial.printf("res:%d\r\n", res);

    ioCon1.portMode0(ALLOUTPUT);  // Set the port as all output
}

void loop() {
    // write single, the same read
    ioCon1.digitalWrite0(0, LOW);
    ioCon1.digitalWrite0(1, LOW);
    ioCon1.digitalWrite0(2, LOW);
    ioCon1.digitalWrite0(3, LOW);
    ioCon1.digitalWrite0(4, LOW);
    ioCon1.digitalWrite0(5, LOW);
    ioCon1.digitalWrite0(6, LOW);
    ioCon1.digitalWrite0(7, LOW);
    delay(1000);
    ioCon1.digitalWrite0(0, HIGH);
    ioCon1.digitalWrite0(1, HIGH);
    ioCon1.digitalWrite0(2, HIGH);
    ioCon1.digitalWrite0(3, HIGH);
    ioCon1.digitalWrite0(4, HIGH);
    ioCon1.digitalWrite0(5, HIGH);
    ioCon1.digitalWrite0(6, HIGH);
    ioCon1.digitalWrite0(7, HIGH);
    delay(1000);

    // write 0-7 HIGHT
    Serial.println(ioCon1.digitalWritePort0(0xff));
    delay(200);

    // write 0-7 LOW
    Serial.println(ioCon1.digitalWritePort0(0x00));
    delay(200);

    //// write Port, the same read
    for (byte i = 0; i < 8; i++) {
        ioCon1.digitalWritePort0((1 << i));
        delay(200);
    }
}
