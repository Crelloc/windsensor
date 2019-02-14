#include <SoftwareSerial.h> //for xbee shield


static volatile int counter;                       /**< counter for the # of digital pulses that are outputed by the Met1 speed sensor*/
static volatile long rpm;                          /**< revs per min */
static volatile bool rw_flag;
static volatile float V;                           /**< Velocity [miles per hour] */
static volatile int g_cycles = 0;                  /**< Keeps track of the # of cycles for timer1 */
static double sinSum = 0, cosSum = 0;
static int avg_counter = 0;                        /**< counter to compute the average for avg_adc_x and avg_adc_y*/

//For Atmega2560, ATmega32U4, etc.
// XBee's DOUT (TX) is connected to pin 10 (Arduino's Software RX)
// XBee's DIN (RX) is connected to pin 11 (Arduino's Software TX)
SoftwareSerial XBee(10, 11); // RX, TX

static void Broadcast(int* deg, volatile long* rpm){
    XBee.println(*deg);
    XBee.println(*rpm);
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

void setup() {
    /** Open serial communications and wait for port to open: */
    while (!Serial) {
        ; /**< wait for serial port to connect. Needed for native USB port only */
    }
    Serial.begin(9600);
    /** initialize timer1 - 16 bit (65536) */
    noInterrupts();           // disable all interrupts
    TCCR1A  = 0;
    TCCR1B  = 0;
    
    TCNT1   = 49911;            // preload timer
    TCCR1B |= ((1 << CS12)| (1 << CS10)) ;    // 1024 prescaler 
    TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
    interrupts();             // enable all interrupts

/** initialize timer0 - rising edge triggered interrupt - pin 2 */
    {
        const byte interruptPin = 2;
        pinMode(interruptPin, INPUT);
        attachInterrupt(digitalPinToInterrupt(interruptPin), pin_irq_handler, RISING );
    }
    /** initialize variables for interrupts */
    counter = 0;
    rpm     = 0;
    rw_flag = 0;
    V       = 0.0f;
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
        ++avg_counter;

        if(rw_flag){
            int deg;
            rw_flag = !rw_flag;
    
            /**compute average deg*/
            deg = (int)(rad2deg(atan2(sinSum, cosSum)) + 360.0) % 360;
            Broadcast(&deg, &rpm);
            cosSum = 0;
            sinSum = 0;
            avg_counter = 0;
        }
}
