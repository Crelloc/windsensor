/*
Inline load cell Project


*/

#include <Adafruit_MPL115A2.h>
//#include <stdint.h>
//#include <math.h>
//#include <Wire.h>
//#include <SPI.h>
#include <SD.h>
#include "RTClib.h"
#include "HX711.h"

#define DOUTA   4   // USED FOR LOAD CELL ON X-AXIS
#define CLKA    3
#define DOUTB   6   // USED FOR LOAD CELL ON Y-AXIS
#define CLKB    5

HX711 x_scale(DOUTA, CLKA);
HX711 y_scale(DOUTB, CLKB);

#define calibration_factor  998050.0f //Calibration factor for 1kg load cell

Adafruit_MPL115A2 mpl115a2; //SCL analog pin 5, SDA analog pin 4

const byte interruptPin = 2;
volatile int counter; /**< # of pulses */
volatile long rpm;    /**< revs per min */
volatile bool rw_flag;
volatile float V;     /**< Velocity [miles per hour] */
volatile int g_cycles = 0; /**< # of cycles for timer1 */
float avg_adc_x = 0, avg_adc_y = 0; /**< load sensors dat output adc for x and y axis*/
int avg_counter = 0; /**< counter to compute the average for results_x and _y*/
double sinSum = 0, cosSum = 0;
RTC_PCF8523      rtc;

File        dataFile;
const char  FILE_PATH[]   = "datalog.txt";
const int chipSelect      = 10;

/**Wind dir variables*/
const int sensorPin = A3;    /** input value: wind sensor analog */
int sensorValue = 0;  /** variable to store the value coming from the sensor */

void setup(void)
{
        
    /** Open serial communications and wait for port to open: */
    while (!Serial) {
        ; /**< wait for serial port to connect. Needed for native USB port only */
    }
    Serial.begin(57600);

    mpl115a2.begin(); //for pressure and temp sensor
    
    if (! rtc.begin()) {
        Serial.println("Couldn't find RTC");
        while (1);
    }
    if (! rtc.initialized()) {
        Serial.println("RTC is NOT running!");
        // following line sets the RTC to the date & time this sketch was compiled
        //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        // This line sets the RTC with an explicit date & time, for example to set
        // January 21, 2014 at 3am you would call:
        // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    }
      
    
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

    x_scale.set_scale(calibration_factor);
    y_scale.set_scale(calibration_factor);

    
/** initialize timer1 - 16 bit (65536) */
    noInterrupts();           // disable all interrupts
    TCCR1A  = 0;
    TCCR1B  = 0;
    
    TCNT1   = 49911;            // preload timer 16MHz/1024 => Period=4sec
    TCCR1B |= ((1 << CS12)| (1 << CS10)) ;    // 1024 prescaler 
    TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
    interrupts();             // enable all interrupts

/** initialize timer0 - rising edge triggered interrupt - pin 2 */
    attachInterrupt(digitalPinToInterrupt(interruptPin), pin_irq_handler, RISING );

    /** initialize variables for interrupts */
    counter = 0;
    rpm     = 0;
    rw_flag = 0;
    V       = 0.0f;

#define WRITE_TO_SDCARD(text)                                               \
if((dataFile = SD.open(FILE_PATH, FILE_WRITE))){                 \
        dataFile.println(text);                                             \
        dataFile.close();                                                   \
    } else {                                                                \
        Serial.println("Failed to open or create " + String(FILE_PATH));    \
    }                       

    WRITE_TO_SDCARD("time, x_kg, y_kg, dir, rpm, Vel, kPa, celsius");
    Serial.println("time, x_kg, y_kg,  dir, rpm, Vel, kPa, celsius");
}

ISR(TIMER1_OVF_vect)        
{
#define PERIOD_THRESHOLD 6 /** 6 second period*/
    TCNT1 = 49911;
    g_cycles++;
    
    if(PERIOD_THRESHOLD == g_cycles){
        rpm     = counter * 15L;
        rpm    /= 40L;
        V       = (rpm / 16.767f) + 0.6f;
        rw_flag = 1;
        counter = 0;
        g_cycles = 0;
    }
}

void pin_irq_handler()
{
    ++counter;
}

#define deg2rad(deg) ((deg * 71.0) / 4068.0)
#define rad2deg(rad) ((rad * 4068.0) / 71.0)

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

unsigned long MAX_ADC_VAL = (1UL<<23) - 1;

void loop(void)
{   //deg_averaging_test();
//    float lbs_x, lbs_y; /**< load sensor pounds */
//    float x_mV, y_mV; /**< load sensor in milli-volts*/
    String str; /**< string to write to SD card*/
    double deg;
    float pressureKPA = 0, temperatureC = 0;
//   char buf[]
    /** scaling adc values before getting the sums
     *  32767 is the highest positive adc value for ads1115: (2^15 - 1) 
     */
    avg_adc_x    += (x_scale.read() / (float)MAX_ADC_VAL); 
    avg_adc_y    += (y_scale.read() / (float)MAX_ADC_VAL);

    /** read wind dir analog value */
    sensorValue = analogRead(sensorPin);
    /** map dir sensor val from analog 0 to 1013 -> 0 to 360 deg */
    deg = (sensorValue - 0.0) / (1013.0 - 0.0) * (360.0 - 0.0);

    sinSum += sin(deg2rad(deg));
    cosSum += cos(deg2rad(deg));
    
    ++avg_counter;

    if(rw_flag){
      
        rw_flag = !rw_flag;
        DateTime now = rtc.now();

        /*get pressure and temp*/
        mpl115a2.getPT(&pressureKPA,&temperatureC);
        
        /**compute average deg*/
        deg = (int)(rad2deg(atan2(sinSum, cosSum)) + 360.0) % 360;
        /**get the average adc results */
        avg_adc_x /= avg_counter;
        avg_adc_y /= avg_counter;

        /**unscale avg_adc*/
        avg_adc_x *= MAX_ADC_VAL;
        avg_adc_y *= MAX_ADC_VAL;
        
        double kg_x = avg_adc_x / MAX_ADC_VAL;
        double kg_y = avg_adc_y / MAX_ADC_VAL; 
        str = "";
        str += String(now.year(), DEC);
        str += '/';
        str += String(now.month(), DEC);
        str += '/';
        str += String(now.day(), DEC);
        str += " ";
        str += String(now.hour(), DEC);
        str += ':';
        str += String(now.minute(), DEC);
        str += ':';
        str += String(now.second(), DEC);  
        str += ", ";
        str += String(kg_x*100, 4);
        str += ", ";
        str += String(kg_y*100, 4);
        str += ", ";
        /** map dir sensor val from analog 0 to 1013 -> 0 to 360 deg */
        str += String(deg);
        str += ", ";
        str += String(rpm);
        str += ", ";
        str += String(V);
        str += ", ";
        str += String(pressureKPA, 4);
        str += ", ";
        str += String(temperatureC, 1);
        
        WRITE_TO_SDCARD(str);
        Serial.println(str);

        /** initialize variables for averageing*/
        avg_adc_x = 0;
        avg_adc_y = 0;
        cosSum = 0;
        sinSum = 0;
        avg_counter = 0;
        
    }
}
