const byte interruptPin = 2;
volatile int counter; //# of pulses
volatile long rpm;    //revs per min
volatile bool rw_flag;
volatile float V;     //Velocity [miles per hour]

void setup()
{
  
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }
    Serial.begin(57600);
    
    // initialize timer1 - 16 bit 65536
    noInterrupts();           // disable all interrupts
    TCCR1A  = 0;
    TCCR1B  = 0;
    
    TCNT1   = 15625;            // preload timer 16MHz/1024 => Period=4sec
    TCCR1B |= ((1 << CS12)| (1 << CS10)) ;    // 1024 prescaler 
    TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
    interrupts();             // enable all interrupts

    // initialize timer0 - rising edge triggered interrupt - pin 2
    attachInterrupt(digitalPinToInterrupt(interruptPin), pin_irq_handler, RISING );

    // initialize variables
    counter = 0;
    rpm     = 0;
    rw_flag = 0;
    V       = 0.0f;
}

ISR(TIMER1_OVF_vect)        
{
    rpm     = counter * 15L;
    rpm    /= 40L;
    V       = (rpm / 16.767f) + 0.6f;
    rw_flag = 1;
    counter = 0;
}

void pin_irq_handler()
{
    ++counter;
}

void loop() 
{
    if(rw_flag){

        Serial.print(rpm);
        Serial.print(" ");
        Serial.println(V);
        rw_flag = !rw_flag;
    }
}


