/**
 * @file windsensor.ino
 *Inline load cell Project
 *Author: Thomas Turner (thomastdt@gmail.com)
 *Last Modified: 09-10-18
 */

#include <Adafruit_MPL115A2.h>
#include <SPI.h>
#include <SD.h>
#include "RTClib.h"
#include "HX711.h"
#define DOUTA   4                           /*!< digital pin for Hx711: USED FOR LOAD CELL ON X-AXIS */
#define CLKA    3                           /*!< digital pin for Hx711: USED FOR LOAD CELL ON X-AXIS */
#define DOUTB   6                           /*!< digital pin for Hx711: USED FOR LOAD CELL ON Y-AXIS */
#define CLKB    5                           /*!< digital pin for Hx711: USED FOR LOAD CELL ON Y-AXIS */

//! Using HX711 constructor
/*!
 The HX711 load cell amplifier will be connected to digital pins of arduino.
 \param DOUTA the pin number to arduino digital
 \param CLKA the pin number to arduino digital
*/
static HX711 x_scale(DOUTA, CLKA);

//! Using HX711 constructor
/*!
 The HX711 load cell amplifier will be connected to digital pins of arduino.
 \param DOUTA the pin number to arduino digital
 \param CLKA the pin number to arduino digital
*/
static HX711 y_scale(DOUTB, CLKB);

#define calibration_factor  1.0f                   /*!< Calibration factor for load cells using hx711 */

static Adafruit_MPL115A2 mpl115a2;                 

static volatile int counter;                       /**< counter for the # of digital pulses that are outputed by the Met1 speed sensor*/
static volatile long rpm;                          /**< revs per min */
static volatile bool rw_flag;
static volatile float V;                           /**< Velocity [miles per hour] */
static volatile int g_cycles = 0;                  /**< Keeps track of the # of cycles for timer1 */
static double avg_adc_x = 0, avg_adc_y = 0;         /**< load sensors data output adc for x and y axis*/
static int avg_counter = 0;                        /**< counter to compute the average for avg_adc_x and avg_adc_y*/
static double sinSum = 0, cosSum = 0;
static RTC_PCF8523      rtc;

//! Log the measurments to the SD card
/*!
 The HX711 load cell amplifier will be connected to digital pins of arduino.
 \param deg a pointer to a variable that calculated the average for degrees.
*/
static void logToSD(int* deg)
{
    String str;
    float pressureKPA = 0, temperatureC = 0;
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    DateTime now = rtc.now();

    //! get pressure and temperature from mpl115a2 sensor
    /*!
      \param pressureKPA a float passed by reference
      \param temperatureC a float passed by reference
    */
    mpl115a2.getPT(&pressureKPA,&temperatureC);
    
    if(dataFile){
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
        str += String(avg_adc_x, 4);
        str += ", ";
        str += String(avg_adc_y, 4);
        str += ", ";
        str += String(*deg);
        str += ", ";
        str += String(rpm);
        str += ", ";
        str += String(V);
        str += ", ";
        str += String(pressureKPA, 4);
        str += ", ";
        str += String(temperatureC, 1);

        dataFile.println(str);                                             
        dataFile.close();
    } else{
        Serial.println("Failed to open or create datalog.txt");       
    }

     Serial.println(str);

}

void setup(void)
{
    const int chipSelect = 10; /**< pin 10 for the sd chipselect on the adafruit datalogging shield.*/
    
    /** Open serial communications and wait for port to open: */
    while (!Serial) {
        ; /**< wait for serial port to connect. Needed for native USB port only */
    }
    Serial.begin(9600);
    Serial.println("testing!");
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
    {
        const byte interruptPin = 2;
        attachInterrupt(digitalPinToInterrupt(interruptPin), pin_irq_handler, RISING );
    }
    /** initialize variables for interrupts */
    counter = 0;
    rpm     = 0;
    rw_flag = 0;
    V       = 0.0f;

    {
        File dataFile = SD.open("datalog.txt", FILE_WRITE);
        if(dataFile){
            dataFile.println("time, x_kg, y_kg, dir, rpm, Vel, kPa, celsius");
            dataFile.close();
            
        } else{
            Serial.println("Error creating or opening datalog.txt!");
        }
        Serial.println("time, x_kg, y_kg,  dir, rpm, Vel, kPa, celsius");
        
    }
}

//! Timer1 hardware interrupt
/*!
 This interrupt function is called every second (1 hz).
 Every six seconds calculate rpm and velocity for met1 speed sensor.
*/
ISR(TIMER1_OVF_vect)        
{
#define PERIOD_THRESHOLD 6 /**< 6 second period*/
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
//! rising edge triggered interrupt
/*!
  Counts the number of pulses outputed by the Met1 speed sensor
*/
static void pin_irq_handler()
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

#define MAX_ADC_VAL ((1UL<<23) - 1)  /**< Maximum positive value for 24bit*/

void loop(void)
{   
    {
        //! Averaging for load cells
        /*!
          Get digital value from hx711 and scale.
          Scale by Maximum ADC Value for 24 bit resolution.
        */
        avg_adc_x    += (x_scale.read() / (double)MAX_ADC_VAL); 
        avg_adc_y    += (y_scale.read() / (double)MAX_ADC_VAL);
    }

    {
        float deg;
        const int sensorPin = A3;    /**< input value: wind sensor analog */
        
        //! read wind dir analog value and Map*/
        /*! map dir sensor val from analog 0 to 1013 -> 0 to 360 deg.
            Get the sines and cosines for averaging degrees.
        */
        deg = (analogRead(sensorPin) - 0.0f) / (1013.0f - 0.0f) * (360.0f - 0.0f);
        sinSum += sin(deg2rad(deg));
        cosSum += cos(deg2rad(deg));
        ++avg_counter;
    }
    
    if(rw_flag){
        int deg;
        rw_flag = !rw_flag;

        /**compute average deg*/
        deg = (int)(rad2deg(atan2(sinSum, cosSum)) + 360.0) % 360;
        /**get the average adc results */
        avg_adc_x /= avg_counter;
        avg_adc_y /= avg_counter;

        /**unscale avg_adc*/
        avg_adc_x *= MAX_ADC_VAL;
        avg_adc_y *= MAX_ADC_VAL;
        
        logToSD(&deg);
        /** initialize variables for averageing*/
        avg_adc_x = 0;
        avg_adc_y = 0;
        cosSum = 0;
        sinSum = 0;
        avg_counter = 0;
        
    }
    
}
