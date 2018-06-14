#include <Wire.h>
int TCAADDR = 0x70; // Multiplexer address
int FSAADDR = 0x58; // Force Sensor address
int xdata, ydata;
double xforce, yforce; //force in pounds
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}
void setup() {
  Wire.begin(); // Initiate the Wire library
  Serial.begin(9600);
  delay(100);
}
void loop() {
  tcaselect(2); 
  Wire.requestFrom(FSAADDR,2); // Request the transmitted two bytes
  if(Wire.available()<=2) {  // reading in a max of two bytes 
    xdata = Wire.read() << 2; // Reads the data, shift away status bits
  }
  tcaselect(6); 
  Wire.requestFrom(FSAADDR,2); // Request the transmitted two bytes
  if(Wire.available()<=2) {  // reading in a max of two bytes 
    ydata = Wire.read() << 2; // Reads the data, shift away status bits
  }
  if (xdata > 5) { //fix zero value 
    xforce = ((xdata-8.0)/(252-8))*1.5; //data ranges from 8 to 252, 1.5 lb rated force range
    Serial.print("x axis force= ");
    Serial.print(xforce);
    Serial.print(" lbs"); 
  }
 
  if (ydata > 5) { //fix zero value 
    yforce = ((ydata-12.0)/(252-12))*1.5; //data ranges from 12 to 252, 1.5 lb rated force range
    Serial.print("      y axis force= ");
    Serial.print(yforce);
    Serial.println(" lbs"); 
  }


}
