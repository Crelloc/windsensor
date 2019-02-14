# windsensor Arduino 

## Up to date branches
* Digital Branch: Base-plate implementation integrated with Met1 sensor
   * Implementing: 
      * FSAGPDXX1.5LC5B5 digital force sensors
      * Adafruit Data Logging Shield
      * Met 1 wind speed and direction sensors
     * BME280 sensor
     * TCA9548A i2c Multiplexer
     * MCP9808 sensor
     * LM324N Op Amp to decouple a (11 volt max) met 1 to arduino digital input (5 volt max)
* Inline-load-cell: Single rod with 2 load cells attached
   * Code implements Met1 sensor and load cells
* windsensor-met1:
    * Code for the cost-effective windsensor and the met1 windsensor to run separately but interact via XBee
    
