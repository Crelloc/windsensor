/**
 * @file inline.ino
 *Inline load cell Project
 *Author: Thomas Turner (thomastdt@gmail.com)
 *Last Modified: 04-11-19
 *
 *
 */

#include <Adafruit_MPL115A2.h>
#include <SoftwareSerial.h> //for xbee shield
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <HX711.h>
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
HX711 x_scale(DOUTA, CLKA);

//! Using HX711 constructor
/*!
 The HX711 load cell amplifier will be connected to digital pins of arduino.
 \param DOUTA the pin number to arduino digital
 \param CLKA the pin number to arduino digital
*/
HX711 y_scale(DOUTB, CLKB);


Adafruit_MPL115A2 mpl115a2;    
             
//For Atmega2560, ATmega32U4, etc.
// XBee's DOUT (TX) is connected to pin 10 (Arduino's Software RX)
// XBee's DIN (RX) is connected to pin 11 (Arduino's Software TX)
SoftwareSerial XBee(2, 3); // RX, TX


static volatile bool rw_flag;
static volatile int g_cycles = 0;                  /**< Keeps track of the # of cycles for timer1 */
static double avg_adc_x = 0, avg_adc_y = 0;         /**< load sensors data output adc for x and y axis*/
static int avg_counter = 0;                        /**< counter to compute the average for avg_adc_x and avg_adc_y*/
static uint8_t g_Index;
static char g_Buffer[BUF_SIZE];
RTC_PCF8523      rtc;

typedef struct Met1{
  long rpm;
  int  deg;
  double vel;
} Met1;

bool g_beginning = false;
int g_ret = 0;

static int get_string() //read the xbee buffer, return a flag if it's time to execute.
{
    int ret = 0;
    while(XBee.available()){ //while there's a chacter in the Xbee buffer, read.  add buffer to the command global variable.  if end of line character, set flag to 1.
        char c = XBee.read();
        //Serial.write(c);
        if(g_beginning || (c == 'd')){
            if(!g_beginning)
                g_beginning = true;
            if(c != '\r')
                g_Buffer[g_Index] = c; //store character in array
            else {
                g_Buffer[g_Index] = '\0';
                g_Index = 0;
                ret = 1;
                g_beginning = false;
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

    Serial.print("\ndegrees is: "), Serial.println(sys.deg);
    Serial.print("velocity is: "), Serial.println(sys.vel);
    Serial.print("rpm is: "), Serial.println(sys.rpm);
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
static void logToSD()
{
    Met1 sys;
//    char buf[100];
//    char x_adc[16];
//    char y_adc[16];
//    char pressure[16];
//    char tempC[16];
//    char vel[16];
//    float pressureKPA        = 0;
//    float temperatureC       = 0;
//    DateTime now             = rtc.now();
//    byte year                = now.year();
//    byte month               = now.month();
//    byte day                 = now.day();
//    byte hour                = now.hour();
//    byte minute              = now.minute();
//    byte second              = now.second();
//    File dataFile            = SD.open("datalog.txt", FILE_WRITE);

    //! get pressure and temperature from mpl115a2 sensor
    /*!
      \param pressureKPA a float passed by reference
      \param temperatureC a float passed by reference
    */
    //mpl115a2.getPT(&pressureKPA,&temperatureC);
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
//    if(dataFile){
//        dataFile.println(buf);                                         
//        dataFile.close();
//    } else{
//        Serial.println("Failed to open or create datalog.txt");       
//    }
//    str = "";
//    str += String(now.year(), DEC);
//    str += '/';
//    str += String(now.month(), DEC);
//    str += '/';
//    str += String(now.day(), DEC);
//    str += " ";
//    str += String(now.hour(), DEC);
//    str += ':';
//    str += String(now.minute(), DEC);
//    str += ':';
//    str += String(now.second(), DEC);  
//    str += ", ";
//    str += String(avg_adc_x, 4);
//    str += ", ";
//    str += String(avg_adc_y, 4);
//    str += ", ";
//    str += String(sys.deg);
//    str += ", ";
//    str += String(sys.rpm);
//    str += ", ";
//    str += String(sys.vel);
//    str += ", ";
//    str += String(pressureKPA, 4);
//    str += ", ";
//    str += String(temperatureC, 1);
//    Serial.println(str);
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
//    
//    if (! rtc.begin()) {
//        Serial.println("Couldn't find RTC");
//    }
//    if (! rtc.initialized()) {
//        Serial.println("RTC is NOT running!");
//        // following line sets the RTC to the date & time this sketch was compiled
//        //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
//        // This line sets the RTC with an explicit date & time, for example to set
//        // January 21, 2014 at 3am you would call:
//        // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
//    }
//    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
//
//
//    Serial.print("\nInitializing SD card...");
//    /** make sure that the default chip select pin is set to
//     *  output, even if you don't use it:
//     */
//    pinMode(chipSelect, OUTPUT); 
//   
//    if (!SD.begin(chipSelect)) {
//        Serial.println("initialization failed. Things to check:");
//        Serial.println("* is a card inserted?");
//        Serial.println("* is your wiring correct?");
//        Serial.println("* did you change the chipSelect pin to match your shield or module?");
//    } else {
//        Serial.println("Wiring is correct and a card is present.");
//    }
//
    x_scale.set_scale(calibration_factor);
    y_scale.set_scale(calibration_factor);
    x_scale.set_gain(64);
    y_scale.set_gain(64);

    
/** initialize timer1 - 16 bit (65536) */
    noInterrupts();           // disable all interrupts
    TCCR1A  = 0;
    TCCR1B  = 0;
    
    TCNT1   = 49911;            // preload timer; initialize counter
    TCCR1B |= ((1 << CS12)| (1 << CS10)) ;    // 1024 prescaler: 16MHz/1024 - 1
    TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
    interrupts();             // enable all interrupts


//    {
//        File dataFile = SD.open("datalog.txt", FILE_WRITE);
//        if(dataFile){
//            dataFile.println("time, x_kg, y_kg, dir, rpm, Vel, kPa, celsius");
//            dataFile.close();
//            
//        } else{
//            Serial.println("Error creating or opening datalog.txt!");
//        }
//        Serial.println("time, x_kg, y_kg,  dir, rpm, Vel, kPa, celsius");
//        
//    }
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

    g_ret = get_string();
      
    //! Averaging for load cells
    /*!
      Get digital value from hx711 and scale.
      Scale by Maximum ADC Value for 24 bit resolution.
    */
    avg_adc_x    += (x_scale.read() / (double)MAX_ADC_VAL); 
    avg_adc_y    += (y_scale.read() / (double)MAX_ADC_VAL);
    avg_counter++;

    if(rw_flag && g_ret){
        rw_flag = !rw_flag;
        /**get the average adc results */
        avg_adc_x /= avg_counter;
        avg_adc_y /= avg_counter;

        /**unscale avg_adc*/
        avg_adc_x *= MAX_ADC_VAL;
        avg_adc_y *= MAX_ADC_VAL;

        logToSD();
        /** initialize variables for averageing*/
        avg_adc_x = 0;
        avg_adc_y = 0;

        avg_counter = 0;
    } else if(!rw_flag && g_ret){
        memset(g_Buffer, 0, strlen(g_Buffer));
        XBee.flush();
    }
    
}
