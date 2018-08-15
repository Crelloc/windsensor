#include <Wire.h>
#include <SD.h>






void setup(void)
{
        
    /** Open serial communications and wait for port to open: */
    while (!Serial) {
        ; /**< wait for serial port to connect. Needed for native USB port only */
    }
    Serial.begin(9600);
    Serial.print("\nInitializing SD card...");
    // make sure that the default chip select pin is set to
    // output, even if you don't use it:
    pinMode(10, OUTPUT);
    // we'll use the initialization code from the utility libraries
    // since we're just testing if the card is working!
    if (!SD.begin(10)) {
        Serial.println("initialization failed. Things to check:");
        Serial.println("* is a card inserted?");
        Serial.println("* is your wiring correct?");
        Serial.println("* did you change the chipSelect pin to match your shield or module?");
       // return;
    } else {
        Serial.println("Wiring is correct and a card is present.");
    }
File dataFile = SD.open("datalog.txt", FILE_WRITE);
    Serial.println(dataFile);
}
void loop(void) {

}

