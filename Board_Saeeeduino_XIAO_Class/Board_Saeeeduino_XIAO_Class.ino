/*
# Programmer:   Raza Bhatti
# Company:      Softgalaxy Technologies Inc.
# Website:      https://www.linkedin.com/in/softgalaxy
# 
# Observation or Principle to proposed solutions: There are mostly or always multiple ways to achieve same objective.
# 
# Arduino IDE using Saeeeduino XIAO board Program Function: 
# Explores class functionality using Arduino IDE
# 1. Ability to set all of 11 pins as input or output and check pin status later as set previously as input or output.
# 2. I2C communication testing capability.
# 3. SPI communication testing capability.
# 4. Ability to use ADC on board.
# 5. Ability to use DAC on board.
# Reference Links:
# 1. https://wiki.seeedstudio.com/Seeeduino-XIAO/
# 2.
*/

/*-------------------------------------------------------------------------------------------------*/
//If you want class_Saeeduino_XAIO class in a separate file, enable following section
// ThisClass.h or any name of your choice.
//##ifndef ThisClass_h
//##define ThisClass_h
//Enable above lines if moving class in a separate file.
//#include <Arduino.h>      //May be required
/*-------------------------------------------------------------------------------------------------*/
#include "SPI.h"
#include "WIRE.h"

#define pinDAC  0          //pin # for DAC output.
#define pinADC  1          //pin # for ADC input from A1.
#define spi_CS  7          // Uses pin 8 as the spi_SCK, pin 9 as spi_MISO, pin 10 as spi_MOSI.

/*-------------------------------------------------------------------------------------------------*/
//Global variables.
u_int i_pinNo=0;
int i_Analog_Input_Data=0;

//Variables for adjusting DAC output
float x = 0; // Value to take the sin of
float increment = 1;  // default increment= 0.02, value to increment x by each time
int frequency = 440; // Frequency of sine wave

//Variables for I2C
byte i2c_data = 0;

/*-------------------------------------------------------------------------------------------------*/
class class_Saeeduino_XAIO {
  private:
    uint32_t pinState[10];                                    //Array to store pin state as input(1) or output(0)

  public:
    void setPinIO_State(uint32_t pin, uint32_t state);        //Sets Saeeeduino XIAO respective pins I/O state (input or output).
    void setPin_Output(uint32_t pin, uint32_t pinOutput);     //Sets Saeeeduino XIAO respective pin(s) as output.
    int readPin_Input(uint32_t pin);                          //Sets Saeeeduino XIAO respective pin(s) as input.
    uint32_t getPin_State(uint32_t pin);                      //Checks Saeeeduino XIAO respective pins I/O state.    

    void start_SPI();                                            //Sets Saeeeduino XIAO respective pins for SPI communication.
    void send_SPI(char strData[]);                                            //Sets Saeeeduino XIAO respective pins for SPI communication.
                                                              // pin 8 as the SCK pin of SPI, MISO pin of SPI is pin 9,MOSI pin of SPI is pin 10.
    void start_I2C();                                            //Sets Saeeeduino XIAO respective pins for I2C communication.
    void sendI2C(char strData[], uint8_t ui_DeviceAddress);                             //Send data to I2C address
    
                                                                  //Use pin 5 as the SCL pin of I2C, SDA pin of I2C is pin 4.
    int readADC(uint32_t pin);                                //Sets Saeeeduino XIAO respective pins for ADC input, Use pin 6 as the analog pin.
    void writeDAC(uint32_t pin, uint32_t pinOutput);          //Sets Saeeeduino XIAO respective pins for DAC output.
    void setSerialPort();                                     //Sets pin 7 for RX and pin 6 for TX.
    
};
/*-------------------------------------------------------------------------------------------------*/
int class_Saeeduino_XAIO::readADC(uint32_t pin)               //Use pin 6 as the analog input pin.
{
    pinMode(pin, INPUT);                                      //state as INPUT=1, or OUTPUT=0
    analogReadResolution(pin);                                 // Set analog input resolution to max, 12-bits
    i_Analog_Input_Data=analogRead(pin);
    return(i_Analog_Input_Data);
}
/*-------------------------------------------------------------------------------------------------*/
void class_Saeeduino_XAIO::writeDAC(uint32_t pin, uint32_t ui_Output_Value)     //
{
   analogWriteResolution(pin); // Set analog out resolution to max, 10-bits
}
/*-------------------------------------------------------------------------------------------------*/
void class_Saeeduino_XAIO::setSerialPort()     //XIAO pins 6 and 7 used for serial communication.
{
    Serial1.begin(115200);
    while (!Serial1);
}
/*-------------------------------------------------------------------------------------------------*/
void class_Saeeduino_XAIO::start_I2C()     //XIAO pins 6 and 7 used for serial communication.
{
    Wire.begin(); // join i2c bus (address optional for master)
}
/*-------------------------------------------------------------------------------------------------*/
void class_Saeeduino_XAIO::sendI2C(char strData[], uint8_t ui_DeviceAddress)     //Send data to I2C channel.
{
  Serial.print("\nSending Data << ");
  Serial.print(strData);                  // Debugging purposes.
  Serial.print(" >> to I2C address ");
  Serial.print(ui_DeviceAddress); // transmit to device #4
  Serial.println(". ");

  Wire.beginTransmission(ui_DeviceAddress); // transmit to device #4
  Wire.write(strData);                      // sends data  
  Wire.endTransmission();                   // stop transmitting
}
/*-------------------------------------------------------------------------------------------------*/
void class_Saeeduino_XAIO::start_SPI()     //XIAO pins 6 and 7 used for serial communication.
{
   digitalWrite(spi_CS, HIGH);                 // disable Slave Select
   SPI.begin ();
   SPI.setClockDivider(SPI_CLOCK_DIV8);    //divide the clock by 8
}
/*-------------------------------------------------------------------------------------------------*/
void class_Saeeduino_XAIO::send_SPI(char send_Data[])     //XIAO pins 6 and 7 used for serial communication.
{
  char c_Data;
  //Debug printing lines
  Serial.print("\nSending SPI data << ");
  Serial.print(send_Data);
  Serial.println(" >>. ");

  digitalWrite(spi_CS, LOW);                   // enable Slave Select
  // Send data string send_Data[]
  // for (const char * p = "Hello, world!\r" ; c = *p; p++) {
  for (const char *p = send_Data ; c_Data = *p; p++) 
  {
    //Debug printing
    Serial.print(c_Data);
    Serial.print(",");
    SPI.transfer (c_Data);
  }
  Serial.println(".");
  digitalWrite(spi_CS, HIGH);                   // disable Slave Select
}
/*-------------------------------------------------------------------------------------------------*/
void class_Saeeduino_XAIO::setPinIO_State(uint32_t pin, uint32_t state)
{
  pinMode(pin, state);  //state as INPUT=1, or OUTPUT=0
  pinState[pin]=state;                    //Store for later potential enquiries.
}
/*-------------------------------------------------------------------------------------------------*/
void class_Saeeduino_XAIO::setPin_Output(uint32_t pin, uint32_t HighOrLow)
{
  digitalWrite(pin, HighOrLow);       //Sets output pin as low or high.
}
/*-------------------------------------------------------------------------------------------------*/
int class_Saeeduino_XAIO::readPin_Input(uint32_t pinReadState)
{
  return (digitalRead(pinReadState));       //Reads and return input pin state.
}
/*-------------------------------------------------------------------------------------------------*/
u_int32_t class_Saeeduino_XAIO::getPin_State(uint32_t pin)
{
  return(pinState[pin]);    //Return pin state
}
/*-------------------------------------------------------------------------------------------------*/
//#endif        //Enable this line if the class above is moved in a separate file.

class_Saeeduino_XAIO obj1_Saeeduino_XAIO;

void setup() 
{
  //Intialization block
  Serial.begin(9600);         //Saeeeduino serial port baud rate.  

/*  
  //Setting all ODD pins as OUTPUTS(0) and EVEN pins as INPUTS(1)
  obj1_Saeeduino_XAIO.setPinIO_State(0,0);       //Set Pin 1 as output
  obj1_Saeeduino_XAIO.setPinIO_State(1,1);       //Set Pin 1 as input
  obj1_Saeeduino_XAIO.setPinIO_State(2,0);       //Set Pin 1 as input
  obj1_Saeeduino_XAIO.setPinIO_State(3,1);       //Set Pin 1 as output
  obj1_Saeeduino_XAIO.setPinIO_State(4,0);       //Set Pin 1 as input
  obj1_Saeeduino_XAIO.setPinIO_State(5,1);       //Set Pin 1 as output 
  obj1_Saeeduino_XAIO.setPinIO_State(6,0);       //Set Pin 1 as input
  obj1_Saeeduino_XAIO.setPinIO_State(7,1);       //Set Pin 1 as output
  obj1_Saeeduino_XAIO.setPinIO_State(8,0);       //Set Pin 1 as input
  obj1_Saeeduino_XAIO.setPinIO_State(9,1);       //Set Pin 1 as output
  obj1_Saeeduino_XAIO.setPinIO_State(10,0);       //Set Pin 1 as input
*/

  //obj1_Saeeduino_XAIO.setSerialPort();            //Intialize another serial port. Need testing.

  //
  
}
/*-------------------------------------------------------------------------------------------------*/
void loop() 
{
  // Program Start  
  
  /* This block check the pin status of all XIAO pins
  Serial.print("Saeeeduino XIAO Pin ");
  Serial.print(i_pinNo);
  Serial.print("_State is_");
  if(obj1_Saeeduino_XAIO.getPin_State(i_pinNo))  
    Serial.println("INPUT");       //Display pin status as input
  else
    Serial.println("OUTPUT");      //Display pin status as output

  i_pinNo++;
  if(i_pinNo>10)
    i_pinNo=0;

  */

  /*
  //Configure and read from analog channel at pin #6 or any other of your choice, consult XIAO datasheet, user manual.
  i_Analog_Input_Data=obj1_Saeeduino_XAIO.readADC(6);       //Read the digital value from ADC port at pin #6.
  Serial.print("ADC Value= ");
  Serial.println(i_Analog_Input_Data);
  */  

/*-------------------------------------------------------------------------------------------
/* The block of code to test DAC and ADC connectd pins 
  // Generate a voltage value between 0 and 1023 at a pin of your choice. 
  // Let's scale a sin wave between those values:
  // Offset by 511.5, then multiply sin by 511.5.
  int dacVoltage = (int)(511.5 + 511.5 * sin(x));
  x += increment; // Increase value of x

  // Generate a voltage between 0 and 3.3V.
  // 0= 0V, 1023=3.3V, 512=1.65V, etc.
  SerialUSB.print("Writing at DAC pin ");
  SerialUSB.print(pinDAC);
  SerialUSB.print(", ");
  SerialUSB.print(dacVoltage); // Print the voltage.
  SerialUSB.print(" \n");
  
  analogWrite(pinDAC, dacVoltage);

  // Now read A1 (connected to A0 or a pin of your choice), and convert that
  // 12-bit ADC value to a voltage between 0 and 3.3v.
  float voltage = analogRead(pinADC) * 3.3 / 4096.0;
  SerialUSB.print("ADC Voltage at pin ");
  SerialUSB.print(pinADC);
  SerialUSB.print(" is ");
  SerialUSB.print(voltage); // Print the voltage.
  SerialUSB.println(" volts. \n\n");
  
  delay(1); // Delay 1ms
/*-------------------------------------------------------------------------------------------*/

/*
//Test I2C sending function
  obj1_Saeeduino_XAIO.start_I2C();
  obj1_Saeeduino_XAIO.sendI2C("A Test of I2C Communication", 2);
*/

//Test SPI send function
  obj1_Saeeduino_XAIO.start_SPI();
  obj1_Saeeduino_XAIO.send_SPI("Testing SPI communication\r");



  delay(2000);
/*
  //The following block compiles and runs, but needs to be tested.
  Serial1.println("Hello,World");
  delay(1000);
*/  

}
/*-------------------------------------------------------------------------------------------------*/
