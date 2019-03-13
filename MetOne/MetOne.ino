#include <SoftwareSerial.h> //for xbee shield

#define AVG_BUF_SIZE  24 // (6 sec averaging)/(.25sec period) = 24
static double g_avgdeg_buf[AVG_BUF_SIZE] = {};
static double g_avgrpm_buf[AVG_BUF_SIZE] = {};
static double g_avgdeg_sum = 0;
static double g_avgrpm_sum = 0;
static volatile int g_avg_buf_pos = -1;
static volatile int counter;                       /**< counter for the # of digital pulses that are outputed by the Met1 speed sensor*/
static volatile long rpm = 0;                      /**< revs per min */
static volatile bool rw_flag;
static double sinSum = 0, cosSum = 0;
static int recent_deg = 0;
static long recent_rpm = 0;
//For Atmega2560, ATmega32U4, etc.
// XBee's DOUT (TX) is connected to pin 10 (Arduino's Software RX)
// XBee's DIN (RX) is connected to pin 11 (Arduino's Software TX)
SoftwareSerial XBee(8, 9); // RX, TX


/**
 * Compute moving average and add sample to ring buffer
*/
double movingAverage(double *Arr, double *Sum, volatile int pos, int len, double num)
{
    *Sum     = *Sum - Arr[pos] + num;
    Arr[pos] = num;
    return *Sum / len;
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

static void Broadcast(int* deg, volatile long* rpm){
    char buf[100];
    char vel[20];

    sprintf_f((*rpm / 16.767f) + 0.6f, vel);
    sprintf(buf, "d %d r %ld v %s", *deg, *rpm, vel);
    XBee.println(buf);
    Serial.println(buf);
}

//! Timer1 hardware interrupt
/*!
 This interrupt function is called every quarter second (4 hz).
 Every 0.25 seconds calculate rpm for met1 speed sensor.

TIMER1_CAPT_vect:         Timer/Counter1 Capture Event 
TIMER1_COMPA_vect:        Timer/Counter1 Compare Match A 
TIMER1_COMPB_vect:        Timer/Counter1 Compare Match B
TIMER1_OVF_vect:          Timer/Counter1 Overflow
*/
ISR(TIMER1_COMPA_vect)        
{
    rpm     = counter * 240L; // [pulses/.25sec][60sec/min] = [pulses/min]
    rpm    /= 40L;            // [pulses/min][1rev/40pulses]=[rev/min]
    rw_flag = 1;
    counter = 0;
    ++g_avg_buf_pos;

    if(g_avg_buf_pos == AVG_BUF_SIZE){ //end of ring buffer check
        g_avg_buf_pos = 0;
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

void setup() {
    /** Open serial communications and wait for port to open: */
    while (!Serial) {
        ; /**< wait for serial port to connect. Needed for native USB port only */
    }
    Serial.begin(9600);
    XBee.begin(9600);
    /** initialize timer1 - 16 bit (65536)
    * set timer1 to interrupt at 4Hz or 0.25 sec
    */
    noInterrupts();                            // disable all interrupts
    TCCR1A  = 0;
    TCCR1B  = 0;
    TCNT1   = 0;                              // initialize counter value to 0
    // set compare match register for 4hz increments
    OCR1A   = 3905;                          // (16*10^6) / (4*1024) - 1 [must be <65536]
    TCCR1B |= (1 << WGM12);                   // turn on CTC mode
    TCCR1B |= ((1 << CS12)| (1 << CS10)) ;    // 1024 prescaler 
    TIMSK1 |= (1 << OCIE1A);                  // enable timer compare interrupt
    interrupts();                              // enable all interrupts

/** initialize timer0 - rising edge triggered interrupt - pin 20 */
    {
        const byte interruptPin = 2;  //digital pins for interrupts: 2, 3, 18, 19, 20, 21
                                       //pins 2 and 3 with be used for software serial (XBee)
        pinMode(interruptPin, INPUT);
        attachInterrupt(digitalPinToInterrupt(interruptPin), pin_irq_handler, RISING );
    }
    /** initialize variables for interrupts */
    counter = 0;
    rpm     = 0;
    rw_flag = 0;
}

void loop() {
        float deg;
        const int sensorPin = A3;    /**< input value: wind sensor analog */
        
        //! read wind dir analog value and Map*/
        /*! map dir sensor val from analog 0 to 1013 -> 0 to 360 deg.
            Get the sines and cosines for averaging degrees.
        */
        deg = (analogRead(sensorPin) - 0.0f) / (1013.0f - 0.0f) * (360.0f - 0.0f);
        sinSum += sin(deg2rad(deg));
        cosSum += cos(deg2rad(deg));

        if(rw_flag){
            int deg;
            rw_flag = !rw_flag;
    
            /**compute average deg*/
            deg = (int)(rad2deg(atan2(sinSum, cosSum)) + 360.0) % 360;
            //track recent deg value with moving average
            recent_deg = movingAverage(g_avgdeg_buf, &g_avgdeg_sum, g_avg_buf_pos, AVG_BUF_SIZE, (double)deg);
            recent_rpm = movingAverage(g_avgrpm_buf, &g_avgrpm_sum, g_avg_buf_pos, AVG_BUF_SIZE, (double)rpm);

            cosSum = 0;
            sinSum = 0;
        }
        Broadcast(&recent_deg, &recent_rpm);
}
