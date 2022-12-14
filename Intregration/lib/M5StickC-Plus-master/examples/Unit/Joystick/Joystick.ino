#include <M5StickCPlus.h>
#include "Wire.h"

#define JOY_ADDR 0x52

void setup() {
    M5.begin();
    // M5.Lcd.clear();
    // disable the speak noise
    // dacWrite(25, 0);

    Wire.begin();
    M5.Lcd.setRotation(3);
}

uint8_t x_data;
uint8_t y_data;
uint8_t button_data;
char data[100];
void loop() {
    // put your main code here, to run repeatedly:
    Wire.requestFrom(JOY_ADDR, 3);
    if (Wire.available()) {
        x_data      = Wire.read();
        y_data      = Wire.read();
        button_data = Wire.read();
        sprintf(data, "x:%d y:%d button:%d\n", x_data, y_data, button_data);
        Serial.print(data);

        M5.Lcd.setCursor(1, 30, 4);
        M5.Lcd.printf("x:%04d        y:%04d\n", x_data, y_data);
        M5.Lcd.setCursor(1, 60, 4);
        M5.Lcd.printf("button:%d\n", button_data);
    }
    delay(200);
}
