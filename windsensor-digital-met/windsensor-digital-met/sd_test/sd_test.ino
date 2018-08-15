#include <Wire.h>
#include <SPI.h>
#include <SD.h>
const int chipSelect      = 10;
const char  FILE_PATH[]   = "datalog.txt";
void setup() {
  // put your setup code here, to run once:
Serial.print("\nInitializing SD card...");
    // make sure that the default chip select pin is set to
    // output, even if you don't use it:
    pinMode(10, OUTPUT);
    // we'll use the initialization code from the utility libraries
    // since we're just testing if the card is working!
    if (!SD.begin(chipSelect)) {
        Serial.println("initialization failed. Things to check:");
        Serial.println("* is a card inserted?");
        Serial.println("* is your wiring correct?");
        Serial.println("* did you change the chipSelect pin to match your shield or module?");
       // return;
    } else {
        Serial.println("Wiring is correct and a card is present.");
    }
}
#define WRITE_TO_SDCARD(text)                                               \
    if((dataFile = SD.open(FILE_PATH, FILE_WRITE))){                 \
        dataFile.println(text);                                             \
        dataFile.close();                                                   \
    } else {                                                                \
        Serial.println("Failed to open or create " + String(FILE_PATH));    \
    }                       

    WRITE_TO_SDCARD("time, x_lbs, y_lbs, Inside temp (C), Outside temp (C), Pressure (Pa), Humidity (%)");
void loop() {
  // put your main code here, to run repeatedly:

}
