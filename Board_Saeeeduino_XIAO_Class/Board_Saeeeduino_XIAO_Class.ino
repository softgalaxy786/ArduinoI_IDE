/*
# Programmer: 	Raza Bhatti
# Company:	    Softgalaxy Technologies Inc.
# Website:	    https://www.linkedin.com/in/softgalaxy
# 
# Observation or Principle to proposed solutions: There are mostly or always multiple ways to achieve same objective.
# 
# Arduino IDE using Saeeeduino XIAO board Program Function: 
# Explores class functionality using Arduino IDE
# 1. Ability to set all of 11 pins as input or output and check pin status later as set previously as input or output.
# 2. 
*/

//If you want class_Saeeduino_XAIO class in a separate file, enable following section
// ThisClass.h or any name of your choice.
//##ifndef ThisClass_h
//##define ThisClass_h
//Enable above lines if moving class in a separate file.

//#include <Arduino.h>      //May be required

class class_Saeeduino_XAIO {
  private:
    uint32_t pinState[10];    //Array to store pin state as input(1) or output(0)

  public:
    void setPinIO_State(uint32_t pin, uint32_t state);
    void setPin_Output(uint32_t pin, uint32_t pinOutput);
    int readPin_Input(uint32_t pin);
    uint32_t getPin_State(uint32_t pin);
    
};

void class_Saeeduino_XAIO::setPinIO_State(uint32_t pin, uint32_t state)
{
  pinMode(pin, state);  //state as INPUT=1, or OUTPUT=0
  pinState[pin]=state;                    //Store for later potential enquiries.
}
void class_Saeeduino_XAIO::setPin_Output(uint32_t pin, uint32_t HighOrLow)
{
  digitalWrite(pin, HighOrLow);       //Sets output pin as low or high.
}
int class_Saeeduino_XAIO::readPin_Input(uint32_t pinReadState)
{
  return (digitalRead(pinReadState));       //Reads and return input pin state.
}
u_int32_t class_Saeeduino_XAIO::getPin_State(uint32_t pin)
{
  return(pinState[pin]);    //Return pin state
}

//#endif        //Enable this line if the class above is moved in a separate file.

class_Saeeduino_XAIO obj1_Saeeduino_XAIO;

void setup() 
{
  //Intialization block
  Serial.begin(9600);         //Saeeeduino serial port baud rate.  
  
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
}

u_int i_pinNo=0;

void loop() 
{
  // Program Start  
  Serial.print("Saeeeduino XIAO Pin ");
  Serial.print(i_pinNo);
  Serial.print("_State is_");
  if(obj1_Saeeduino_XAIO.getPin_State(i_pinNo))  
    Serial.println("INPUT");       //Display pin status as input
  else
    Serial.println("OUTPUT");      //Display pin status as output
  
  delay(2000);
  i_pinNo++;
  if(i_pinNo>10)
    i_pinNo=0;

}
