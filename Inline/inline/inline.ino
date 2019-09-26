/**
 * @file inline.ino
 *Inline load cell Project
 *Author: Thomas Turner (thomastdt@gmail.com)
 *Last Modified: 07-03-19
 *
 *
 */

#include <Adafruit_MPL115A2.h>
#include <SoftwareSerial.h> //for xbee shield
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include "HX711.h"
#define DOUTA   7                           /*!< digital pin for Hx711: USED FOR LOAD CELL ON X-AXIS */
#define CLKA    6                           /*!< digital pin for Hx711: USED FOR LOAD CELL ON X-AXIS */
#define DOUTB   5                           /*!< digital pin for Hx711: USED FOR LOAD CELL ON Y-AXIS */
#define CLKB    4                           /*!< digital pin for Hx711: USED FOR LOAD CELL ON Y-AXIS */
#define calibration_factor  1.0f                   /*!< Calibration factor for load cells using hx711 */
#define BUF_SIZE 50
//! Using HX711 constructor
/*!
 The HX711 load cell amplifier will be connected to digital pins of arduino.
 \param DOUTA the pin number to arduino digital
 \param CLKA the pin number to arduino digital
*/
//HX711 x_scale(DOUTA, CLKA);

//! Using HX711 constructor
/*!
 The HX711 load cell amplifier will be connected to digital pins of arduino.
 \param DOUTA the pin number to arduino digital
 \param CLKA the pin number to arduino digital
*/
//HX711 y_scale(DOUTB, CLKB);


//Adafruit_MPL115A2 mpl115a2;    
             
//For Atmega2560, ATmega32U4, etc.
// XBee's DOUT (TX) is connected to pin 11 (Arduino's Software RX)
// XBee's DIN (RX) is connected to pin 3 (Arduino's Software TX)
// Only pins available for RECEIVE (TRANSMIT can be on any pin):
// (I've deliberately left out pin mapping to the Hardware USARTs - seems senseless to me)
// Pins: 10, 11, 12, 13,  50, 51, 52, 53,  62, 63, 64, 65, 66, 67, 68, 69
// The XBee shield dout and din is hardwired to pins 2 (RX) and 3 (DX): So for Mega,
// to use pin 11 for RX, tie pins 2 and 11 together.
//
SoftwareSerial XBee(11, 3); // RX, TX (Arduino)


static volatile bool rw_flag;
static volatile int g_cycles = 0;                  /**< Keeps track of the # of cycles for timer1 */
static double avg_adc_x = 0, avg_adc_y = 0;         /**< load sensors data output adc for x and y axis*/
static int avg_counter = 0;                        /**< counter to compute the average for avg_adc_x and avg_adc_y*/
static uint8_t g_Index;
static char g_Buffer[BUF_SIZE];
RTC_PCF8523      rtc;
static unsigned long g_sum = 0;
static bool g_ctag = false;

typedef struct Met1{
  long rpm;
  int  deg;
  double vel;
  float temp, pressure, alt, hum;
} Met1;

bool g_beginning = false;
int g_ret = 0;


static int checksum(unsigned long mysum, char * data){
    int ret = 0;
    char * c;
    unsigned long sum;

    c = strchr(data, 'c'); //find c tag in data string
    if (c == NULL){
        Serial.println("c tag not found!");
        return ret;
    }
    c++; //ignore 'c'
    sum = atol(c); //get sum value from data
    //Serial.print("sum: "); Serial.println(sum);
    if(sum != mysum) //if checksum doesn't match return false
        return ret;

    return 1;
}

static void TEST(void){
    char num[] = "123";
    char str[] = "c 123";
    char c = ' ';
    unsigned long sum = 123;
    Serial.println((long)c);
    Serial.println(atol(num));
    Serial.println(checksum(sum, str));
    while(1);
}
static int get_string() //read the xbee buffer, return a flag if it's time to execute.
{
    int ret = 0;

    while(XBee.available()){ //while there's a chacter in the Xbee buffer, read.  add buffer to the command global variable.  if end of line character, set flag to 1.
        char c = XBee.read();
        //Serial.write(c);
        if(g_beginning || (c == 'd')){
            if(!g_beginning)
                g_beginning = true;
            if(c != '\n'){
                g_Buffer[g_Index] = c; //store character in array
                if(c == 'c')
                    g_ctag = true;
                if(g_ctag == false)
                    g_sum = g_sum + c;
            }
            else {
                g_Buffer[g_Index] = '\0';
                g_Index = 0;
                ret = 1;
                g_beginning = false;
                g_ctag = false;
                g_sum = ~(g_sum - ' '); //remove space (that's before the ctage, ie, " c 123") from calculation and then invert g_sum
                break;
            }
            ++g_Index;
            if(g_Index>=BUF_SIZE){
                g_Index = BUF_SIZE - 1; //just prevent buffer overflow
            }
        }
  }
  return ret;
}


static Met1 GetMet1Measurements(){
    Met1 sys;
    char * ptr_to_g_Buffer = g_Buffer; //get buffer location
    Serial.println(ptr_to_g_Buffer);
    
    ptr_to_g_Buffer+=1; //ignore d
    sys.deg = atoi(ptr_to_g_Buffer); //convert deg string to integer
    while(*ptr_to_g_Buffer != 'v'){ptr_to_g_Buffer+=1;} //ignore rpm value
    
    ptr_to_g_Buffer+=1; //ignore v
    sys.vel = atof(ptr_to_g_Buffer); //convert vel string to float value
    while(*ptr_to_g_Buffer != 'r'){ptr_to_g_Buffer+=1;} //ignore deg value
    
    ptr_to_g_Buffer+=1; //ignore r
    sys.rpm = atol(ptr_to_g_Buffer); //convert rpm string to long
    while(*ptr_to_g_Buffer != 't'){ptr_to_g_Buffer+=1;} //ignore rpm value
     
    ptr_to_g_Buffer+=1; //ignore t
    sys.temp = atof(ptr_to_g_Buffer); //convert temp string to float
    while(*ptr_to_g_Buffer != 'p'){ptr_to_g_Buffer+=1;} //ignore temp value
    
    ptr_to_g_Buffer+=1; //ignore p
    sys.pressure = atof(ptr_to_g_Buffer); //convert vel string to float value
    while(*ptr_to_g_Buffer != 'a'){ptr_to_g_Buffer+=1;} //ignore pressure value
    
    ptr_to_g_Buffer+=1; //ignore a
    sys.alt = atof(ptr_to_g_Buffer); //convert altitude string to float
    while(*ptr_to_g_Buffer != 'h'){ptr_to_g_Buffer+=1;} //ignore altitude value

    ptr_to_g_Buffer+=1; //ignore h
    sys.hum = atof(ptr_to_g_Buffer); //convert altitude string to float


    Serial.print("\ndegrees is: "), Serial.println(sys.deg);
    Serial.print("\nvelocity is: "), Serial.println(sys.vel);
    Serial.print("\nrpm is: "), Serial.println(sys.rpm);
    Serial.print("\ntemp is: "), Serial.println(sys.temp);
    Serial.print("\npressure is: "), Serial.println(sys.pressure);
    Serial.print("\naltitude is: "), Serial.println(sys.alt);
    Serial.println("\nhumidity is: "), Serial.println(sys.hum);

    return sys;
}

static void sprintf_f(float fval, char *c)
{
    char *tmpSign = (fval < 0) ? (char*)"-"   : (char*)"";
    float tmpVal  = (fval < 0) ? -fval : fval;

    int tmpInt1   = tmpVal;
    float tmpFrac = tmpVal - tmpInt1;
    int tmpInt2 = trunc(tmpFrac * 10000);

    sprintf(c, "%s%d.%04d", tmpSign, tmpInt1, tmpInt2);
   
}
//! Log the measurments to the SD card
/*!
 The HX711 load cell amplifier will be connected to digital pins of arduino.
 \param deg a pointer to a variable that calculated the average for degrees.
*/
static void logToSD(double avg_adc_x, double avg_adc_y)
{
    Met1 sys;
//    char buf[100];
//    char x_adc[16];
//    char y_adc[16];
//    char pressure[16];
//    char tempC[16];
//    char vel[16];
    float pressureKPA        = 0;
    float temperatureC       = 0;
    DateTime now             = rtc.now();
    uint16_t year            = now.year();
    byte month               = now.month();
    byte day                 = now.day();
    byte hour                = now.hour();
    byte minute              = now.minute();
    byte second              = now.second();
    File dataFile            = SD.open("datalog.txt", FILE_WRITE);

    //! get pressure and temperature from mpl115a2 sensor
    /*!
      \param pressureKPA a float passed by reference
      \param temperatureC a float passed by reference
    */
//    mpl115a2.getPT(&pressureKPA,&temperatureC);
    sys = GetMet1Measurements();
//    sprintf_f(avg_adc_x, x_adc);
//    sprintf_f(avg_adc_y, y_adc);
//    sprintf_f(pressureKPA, pressure);
//    sprintf_f(temperatureC, tempC);
//    sprintf_f(sys.vel, vel);
//
//    sprintf(buf,"%04d/%02d/%02d %02d:%02d:%02d, "
//                "%s, "
//                "%s, "
//                "%d, "
//                "%ld, "
//                "%s, "
//                "%s, "
//                "%s, "
//                ,year, month, day, hour, minute, second,
//                x_adc, y_adc, sys.deg, sys.rpm, vel,
//                pressure, tempC);
//
//    Serial.println(buf);
    if(dataFile){
        dataFile.print(year, DEC);
        dataFile.print('/');
        dataFile.print(month, DEC);
        dataFile.print('/');
        dataFile.print(day, DEC);
        dataFile.print(" ");
        dataFile.print(hour, DEC);
        dataFile.print(':');
        dataFile.print(minute, DEC);
        dataFile.print(':');
        dataFile.print(second, DEC);  
        dataFile.print(", ");
        dataFile.print(avg_adc_x, 4);
        dataFile.print(", ");
        dataFile.print(avg_adc_y, 4);
        dataFile.print(", ");
        dataFile.print(sys.deg);
        dataFile.print(", ");
        dataFile.print(sys.rpm);
        dataFile.print(", ");
        dataFile.print(sys.vel);
        dataFile.print(", ");
        dataFile.print(pressureKPA, 4);
        dataFile.print(", ");
        dataFile.print(sys.pressure, 4);
        dataFile.print(", ");
        dataFile.print(temperatureC, 1);
        dataFile.print(", ");
        dataFile.print(sys.temp, 4);
        dataFile.print(", ");
        dataFile.print(sys.alt, 4);
        dataFile.print(", ");
        dataFile.print(sys.hum, 4);
        dataFile.close();
    } else{
        Serial.println("Failed to open or create datalog.txt");       
    }

    Serial.print(year, DEC);
    Serial.print('/');
    Serial.print(month, DEC);
    Serial.print('/');
    Serial.print(day, DEC);
    Serial.print(" ");
    Serial.print(hour, DEC);
    Serial.print(':');
    Serial.print(minute, DEC);
    Serial.print(':');
    Serial.print(second, DEC);  
    Serial.print(", ");
    Serial.print(avg_adc_x, 4);
    Serial.print(", ");
    Serial.print(avg_adc_y, 4);
    Serial.print(", ");
    Serial.print(sys.deg);
    Serial.print(", ");
    Serial.print(sys.rpm);
    Serial.print(", ");
    Serial.print(sys.vel);
    Serial.print(", ");
    Serial.print(pressureKPA, 4);
    Serial.print(", ");
    Serial.print(sys.pressure, 4);
    Serial.print(", ");
    Serial.print(temperatureC, 1);
    Serial.print(", ");
    Serial.print(sys.temp, 4);
    Serial.print(", ");
    Serial.print(sys.alt, 4);
    Serial.print(", ");
    Serial.print(sys.hum, 4);

}

void setup(void)
{
    const int chipSelect = 10; /**< pin 10 for the sd chipselect on the adafruit datalogging shield.*/
    
    /** Open serial communications and wait for port to open: */
    while (!Serial) {
        ; /**< wait for serial port to connect. Needed for native USB port only */
    }
    Serial.begin(9600);
    XBee.begin(9600);
    Serial.println("testing!");
//    mpl115a2.begin(); //for pressure and temp sensor
    
    if (! rtc.begin()) {
        Serial.println("Couldn't find RTC");
    }
    if (! rtc.initialized()) {
        Serial.println("RTC is NOT running!");
        // following line sets the RTC to the date & time this sketch was compiled
        //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        // This line sets the RTC with an explicit date & time, for example to set
        // January 21, 2014 at 3am you would call:
        // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    }
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));


    Serial.print("\nInitializing SD card...");
    /** make sure that the default chip select pin is set to
     *  output, even if you don't use it:
     */
    pinMode(chipSelect, OUTPUT); 
   
    if (!SD.begin(chipSelect)) {
        Serial.println("initialization failed. Things to check:");
        Serial.println("* is a card inserted?");
        Serial.println("* is your wiring correct?");
        Serial.println("* did you change the chipSelect pin to match your shield or module?");
    } else {
        Serial.println("Wiring is correct and a card is present.");
    }

    //x_scale.set_scale(calibration_factor);
    //y_scale.set_scale(calibration_factor);
    //x_scale.set_gain(64);
    //y_scale.set_gain(64);

    
/** initialize timer1 - 16 bit (65536) */
    noInterrupts();           // disable all interrupts
    TCCR1A  = 0;
    TCCR1B  = 0;
    
    TCNT1   = 49911;            // preload timer; initialize counter
    TCCR1B |= ((1 << CS12)| (1 << CS10)) ;    // 1024 prescaler: 16MHz/1024 - 1
    TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
    interrupts();             // enable all interrupts


    {
        File dataFile = SD.open("datalog.txt", FILE_WRITE);
        if(dataFile){
            dataFile.println("time, x_kg, y_kg,  dir, rpm, Vel, Inline kPa, Met1 pressure"
                                    "Inline celsius, Met1 temp, Met1 altitude, Met1 humidity");
            dataFile.close();
            
        } else{
            Serial.println("Error creating or opening datalog.txt!");
        }
        Serial.println("time, x_kg, y_kg,  dir, rpm, Vel, Inline kPa, Met1 pressure"
                                    "Inline celsius, Met1 temp, Met1 altitude, Met1 humidity");
        
    }
}

//! Timer1 hardware interrupt
/*!
 This interrupt function is called every second (1 hz).
 Every six seconds calculate rpm and velocity for met1 speed sensor.
*/
ISR(TIMER1_OVF_vect)        
{
#define PERIOD_THRESHOLD 2 /**< 6 second period*/
    TCNT1 = 49911;
    g_cycles++;
    
    if(PERIOD_THRESHOLD == g_cycles){
        rw_flag = 1;
        g_cycles = 0;
    }
}

//void deg_averaging_test()
//{
//    int i;
//    double deg[] = {360,270};
//    double sinSum = 0;
//    double cosSum = 0;
//    int result;
//    
//    for(i=0; i< 2; ++i){
//
//        sinSum += sin(deg2rad(deg[i]));
//        cosSum += cos(deg2rad(deg[i]));
//          
//    }
//     result = (int)(rad2deg(atan2(sinSum, cosSum)) + 360.0) % 360;
//     Serial.println(result);
//     while(1){}
//}

#define MAX_ADC_VAL ((1UL<<23) - 1)  /**< Maximum positive value for 24bit*/

void loop(void)
{   
    //TEST();
    g_ret = get_string();
    if(g_ret == 1){
        g_ret = checksum(g_sum, g_Buffer);
    }
    //! Averaging for load cells
    /*!
      Get digital value from hx711 and scale.
      Scale by Maximum ADC Value for 24 bit resolution.
    */
    //avg_adc_x    += (x_scale.read() / (double)MAX_ADC_VAL);
    //avg_adc_y    += (y_scale.read() / (double)MAX_ADC_VAL);
    avg_counter++;

    if(rw_flag && g_ret){
        rw_flag = !rw_flag;
        /**get the average adc results */
        avg_adc_x /= avg_counter;
        avg_adc_y /= avg_counter;

        /**unscale avg_adc*/
        avg_adc_x *= MAX_ADC_VAL;
        avg_adc_y *= MAX_ADC_VAL;

        logToSD(avg_adc_x, avg_adc_y);
        /** initialize variables for averageing*/
        avg_adc_x = 0;
        avg_adc_y = 0;

        avg_counter = 0;
    } else if(!rw_flag && g_ret){//when data is received by met1 but it's not time to log
        memset(g_Buffer, 0, strlen(g_Buffer));
        XBee.flush();
        //reset global variables for checksum
        g_sum = 0;
    }
    
}
