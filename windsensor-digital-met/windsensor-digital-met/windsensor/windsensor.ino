//#define DEPLOYMENT
#include <Wire.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "RTClib.h" 
#include "Adafruit_MCP9808.h"
#define SEALEVELPRESSURE_HPA (1013.25)

static Adafruit_BME280 bme; 
//static int MCPADDR = 0x18; // Inside Temp Sensor address

static volatile int counter;                         /**< # of pulses */
static volatile float rpm;                           /**< revs per min */
static volatile bool rw_flag;
static volatile float V;                             /**< Velocity [miles per hour] */
static volatile int g_cycles = 0;                    /**< # of cycles for timer1 */
static float avg_results_x = 0, avg_results_y = 0;   /**< load sensors values [voltage] for x and y axis*/
static int avg_counter = 0; 
static double sinSum = 0, cosSum = 0;

static RTC_PCF8523      rtc;

// Create the MCP9808 temperature sensor object
static Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

static void tcaselect(uint8_t i) {
  int TCAADDR = 0x70; // Multiplexer address

  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}


static void logToSD(float* lbs_x, float* lbs_y, uint16_t* deg)
{
    String str;
    File dataFile;
    DateTime now = rtc.now();

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
    str += String(*lbs_x, 5);
    str += ", ";
    str += String(*lbs_y, 5);
    str += ", ";
    str += String(*deg);
    str += ", ";
    str += String(rpm);
    str += ", ";
    str += String(V);
    str += ", ";
    str += String(tempsensor.readTempC()); 
    str += ", "; 
    str += String(bme.readTemperature());
    str += ", ";
    str += String(bme.readPressure()/ 100.0);
    str += ", ";
    str += String(bme.readHumidity());

    dataFile = SD.open("datalog.txt", FILE_WRITE);
    if(dataFile){
        dataFile.println(str);                                             
        dataFile.close();
    } else{
        Serial.println("Failed to open or create datalog.txt");       
    }

     Serial.println(str);

}
void setup(void)
{
    File dataFile;
    const byte interruptPin = 2;
    const int chipSelect    = 4;

    
    /** Open serial communications and wait for port to open: */
    while (!Serial) {
        ; /**< wait for serial port to connect. Needed for native USB port only */
    }
    Serial.begin(9600);

    if (!bme.begin()) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        //while (1);
    }

    if (!tempsensor.begin()) {
        Serial.println("Couldn't find MCP9808!");
        //while (1);
    }

    if (!rtc.begin()) {
        Serial.println("Couldn't find RTC");
        //while (1);
    }

    if (!rtc.initialized()) {
        Serial.println("RTC is NOT running!");
        // following line sets the RTC to the date & time this sketch was compiled
        // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        // This line sets the RTC with an explicit date & time, for example to set
        // January 21, 2014 at 3am you would call:
        // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    }

    
    // make sure that the default chip select pin is set to
    // output, even if you don't use it:
    pinMode(chipSelect, OUTPUT);
    // we'll use the initialization code from the utility libraries
    // since we're just testing if the card is working!

    Serial.print("\nInitializing SD card...");
    if (!SD.begin(chipSelect)) {
        Serial.println("initialization failed. Things to check:");
        Serial.println("* is a card inserted?");
        Serial.println("* is your wiring correct?");
        Serial.println("* did you change the chipSelect pin to match your shield or module?");
    } else {
        Serial.println("Wiring is correct and a card is present.");
    }

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

    dataFile = SD.open("datalog.txt", FILE_WRITE);
    {
        String header = "time, x_lbs, y_lbs, dir, rpm, Vel, Inside temp (C), Outside temp (C), Pressure (Pa), Humidity (%)";

        if (dataFile){                  
            dataFile.println(header);                                             
            dataFile.close();                                                   
        }                      
    
        Serial.println(header);
    }

}

ISR(TIMER1_OVF_vect)        
{
#define PERIOD_THRESHOLD 6 /** 6 second period*/
    TCNT1 = 49911;
    g_cycles++;
    
    if(PERIOD_THRESHOLD == g_cycles){
        rpm     = counter * 15.0f;
        rpm    /= 40.0f;
        V       = (rpm / 16.767f) + 0.6f;
        rw_flag = 1;
        counter = 0;
        g_cycles = 0;
    }
}

static void pin_irq_handler()
{
    ++counter;
}

#define deg2rad(deg) ((deg * 71.0) / 4068.0)
#define rad2deg(rad) ((rad * 4068.0) / 71.0)

#define MAX_FORCE_VAL  ((1U<<14) - 1)

static void readforcesensors()
{
    int FSAADDR = 0x58; // Force Sensor address

    tcaselect(2); 
    Wire.requestFrom(FSAADDR,2); // Request the transmitted two bytes
    if(Wire.available()<=2) {  // reading in a max of two bytes 
      avg_results_x += ((Wire.read() << 2) / (float)MAX_FORCE_VAL); // Reads the data, shift away status bits
    }
    tcaselect(6); 
    Wire.requestFrom(FSAADDR,2); // Request the transmitted two bytes
    if(Wire.available()<=2) {  // reading in a max of two bytes 
      avg_results_y += ((Wire.read() << 2) / (float)MAX_FORCE_VAL); // Reads the data, shift away status bits
    }
}

void loop(void)
{
    
    readforcesensors();
    
    {/**read wind dir analog value and average dir*/
        double deg;
        //
        const int sensorPin = A3;
        deg = (analogRead(sensorPin) - 0.0) / (1013.0 - 0.0) * (360.0 - 0.0);
        sinSum += sin(deg2rad(deg));
        cosSum += cos(deg2rad(deg));
        ++avg_counter;
    }

    

    if(rw_flag){
          
        uint16_t deg = 0;
        float lbs_x, lbs_y; /**< load sensor pounds */
                
        rw_flag = !rw_flag;
        
        {/**get the average load cell results*/        
            avg_results_x /= avg_counter;
            avg_results_y /= avg_counter;
    
            /**unscale*/
            avg_results_x *= MAX_FORCE_VAL;
            avg_results_y *= MAX_FORCE_VAL;
    
            lbs_x = ((avg_results_x-8.0)/(252-8))*1.5; //data ranges from 8 to 252, 1.5 lb rated force range
            lbs_y = ((avg_results_y-12.0)/(252-12))*1.5; //data ranges from 12 to 252, 1.5 lb rated force range
        }

        /**compute average deg*/
        deg = (int)(rad2deg(atan2(sinSum, cosSum)) + 360.0) % 360;
        
        logToSD(&lbs_x, &lbs_y, &deg);

        /**initilize*/
        avg_results_x = 0;
        avg_results_y = 0;
        cosSum = 0;
        sinSum = 0;
        avg_counter = 0;
        
     }
}
