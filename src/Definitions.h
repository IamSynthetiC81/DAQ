#include "ErrorHandler/ErrorHandler.h"
#include "SERCOMM/SERCOMM.h"

#include "Definitions/Packet.h"

uint16_t TARGET_SAMPLING_RATE = 500;
volatile uint8_t ADC_MUX_SELECT = 0x00;

//////////////////////////////////////////
////////////    INIT   ///////////////////
//////////////////////////////////////////

bool initTimers(uint16_t  freq){
  cli();                                                                        // Disable all interrupts                  

  // Calculate the top value for Timer1
  unsigned int topValue = (F_CPU / (freq * 8)) - 1;

  // Set the Timer1 mode to CTC (Clear Timer on Compare Match)
  TCCR1A &= ~(1 << WGM10);
  TCCR1A &= ~(1 << WGM11);
  TCCR1B |= (1 << WGM12);
  TCCR1B &= ~(1 << WGM13);

  // Set the prescaler to 8
  TCCR1B &= ~(1 << CS10);
  TCCR1B |= (1 << CS11);
  TCCR1B &= ~(1 << CS12);

  // Set the top value
  OCR1A = topValue;

  /* Init timer 5 for use as a counter  */
  TCCR5A = 0;                                                                   // Init Timer3
  TCCR5B = 0;                                                                   // Init Timer3
  TIMSK5 = 0;                                                                   // Timer/Counter3 Interrupt Mask Register
  TCNT5 = 0;

  
  TCCR5A &= ~(1 << WGM50);                                                      // Normal
  TCCR5A &= ~(1 << WGM51);                                                      // Normal
  TCCR5B &= ~(1 << WGM51);                                                      // Normal
  TCCR5B &= ~(1 << WGM52);                                                      // Normal
  TCCR5B &= ~(1 << WGM53);                                                      // Normal

  TCCR5B &= ~(1 << ICES5);                                                      // Falling edge trigger
  
  TCCR5B |= (1 << CS52);                                                        // Set CS32 to 1
  TCCR5B |= (1 << CS51);                                                        // Set CS31 to 1          
  TCCR5B &= ~(1 << CS50);                                                       // Set CS30 to 0     

  // Enable the Input Capture Noise Canceler
  TCCR5B |= (1 << ICNC5);   

  sei();                                                                        // Enable all interrupts                                                                 

  return true;  
} 

void ADC_init(){
  ADMUX |= (1 << REFS0);                                                      // Set the reference voltage to AVCC
  
  ADCSRB = (ADCSRB & 0x70) | (1 << ADTS2) | (1 << ADTS1);                     // Sets Trigger source to Timer1 Overflow
  DIDR0 = 0xff;                                                               // Disable Login Inputs from Analog Pins
  DIDR2 = 0xff;                                                               // Disable Login Inputs from Analog Pins

  // set an interrupt for when the conversion is complete 
  ADCSRA |= (1 << ADIE);
  ADCSRA |= (1 << ADEN);                                                          // Enable the ADC
}

//////////////////////////////////////////
////////////    FLAGS  ///////////////////
//////////////////////////////////////////

static volatile bool SAMPLE_WINDOW = false;
static volatile bool ADC_WINDOW = false;
static volatile bool recording = false;
static volatile long StartTime = 0;

//////////////////////////////////////////
////////////    LOG     //////////////////
//////////////////////////////////////////
Packet packt = Packet();

//////////////////////////////////////////
///////    ERROR REGISTERS  //////////////
//////////////////////////////////////////

ErrorRegister ERROR_TIMER;
ErrorRegister ERROR_ADC;
ErrorRegister ERROR_IMU;
ErrorRegister ERROR_GPS;
ErrorRegister ERROR_SD;
ErrorRegister ERROR_SERIAL;

ErrorRegister errorRegisters[] = {
  ERROR_TIMER,
  ERROR_ADC,
  ERROR_IMU,
  ERROR_GPS,
  ERROR_SD,
  ERROR_SERIAL
};

//////////////////////////////////////////
////////    ERROR HANDLER  ///////////////
//////////////////////////////////////////

ErrorHandler* errorHandler = new ErrorHandler(errorRegisters, 6);

//////////////////////////////////////////
///////    LOG EXPORT FUNCTIONS    ///////
//////////////////////////////////////////

inline void __attribute__ ((always_inline)) SerialPrintLog(){
  char buffer[124];
  size_t len = packt.toChar(buffer);
  Serial.println(buffer);
}
inline void __attribute__ ((always_inline)) WriteToExternalMem(){
  uint8_t buffer[sizeof(Packet)];
  // LogToBuffer(buffer);

  for (int i = 0 ; i < sizeof(buffer) ; i++ ){
    PORTA = buffer[i];
    // PORTA = (i % 2) == 0 ? 0x01 : 0x02;
    PORTL = 0x01;
    // delayMicroseconds(1);
    PORTL = 0x00;
    // delayMicroseconds(1);
  }
  PORTA = 0x00;
}
inline void __attribute__ ((always_inline)) WriteToSD(){  
}
inline void __attribute__ ((always_inline)) VoidExportFunc(){
  return;
}
inline void __attribute__ ((always_inline)) (*exportFunc)() = VoidExportFunc;

//////////////////////////////////////////
///////   SERIAL COMMANDER  //////////////
//////////////////////////////////////////

char COMMAND_START[] = "Start";
char COMMAND_STOP[] = "Stop";
char COMMAND_RESET[] = "Reset";
char COMMAND_SERIAL_ENABLE[] = "EnableSerial";
char COMMAND_SERIAL_DISABLE[] = "DisableSerial";
char COMMAND_PARALLEL_ENABLE[] = "EnableParallel";
char COMMAND_PARALLEL_DISABLE[] = "DisableParallel";
char COMMAND_FREQUENCY_GET[] = "getSamplingFrequency";
char COMMAND_FREQUENCY_SET[] = "setSamplingFrequency";
char COMMAND_SD_ENABLE[] = "EnableSD";
char COMMAND_SD_DISABLE[] = "DisableSD";

void FUNC_START_SAMPLING(const int argc, char *argv[]){
  recording = true;
  TIMSK1 |= (1 << OCIE1A);                                                    // Enable the Timer1 compare interrupt A
  StartTime = millis();

  // digitalWrite(LED_BUILTIN, HIGH);
  PORTB |=  (1 << PIN7);
}
void FUNC_STOP_SAMPLING(const int argc, char *argv[]){
  recording = false;

  // digitalWrite(LED_BUILTIN, LOW);
  PORTB &= !(1 << PIN7);
}
void FUNC_RESET(const int argc, char *argv[]){
  asm volatile ("  jmp 0");
}
void FUNC_ENABLE_SERIAL(const int argc, char *argv[]){
  exportFunc = SerialPrintLog;
}
void FUNC_DISABLE_SERIAL(const int argc, char *argv[]){
  exportFunc = VoidExportFunc;
}
void FUNC_ENABLE_PARALLEL(const int argc, char *argv[]){
  // exportFunc = ExportParallel;
}
void FUNC_DISABLE_PARALLEL(const int argc, char *argv[]){
  exportFunc = VoidExportFunc;
}
void FUNC_GET_FREQUENCY(const int argc, char *argv[]){
  Serial.print("Sampling Frequency: ");
  Serial.println(TARGET_SAMPLING_RATE);
}
void FUNC_SET_FREQUENCY(const int argc, char *argv[]){
  TARGET_SAMPLING_RATE = atoi(argv[1]);

  initTimers(TARGET_SAMPLING_RATE);

  Serial.print("Sampling Frequency set to: ");
  Serial.println(TARGET_SAMPLING_RATE);
}
void FUNC_ENABLE_SD(const int argc, char *argv[]){
  exportFunc = WriteToSD;
}
void FUNC_DISABLE_SD(const int argc, char *argv[]){
  exportFunc = VoidExportFunc;
}

command_t commands[] = {
  initCommand(FUNC_START_SAMPLING, COMMAND_START, "Starting sampling"),
  initCommand(FUNC_STOP_SAMPLING, COMMAND_STOP, "Stoping sampling"),
  initCommand(FUNC_RESET, COMMAND_RESET, "Resetting the device"),
  initCommand(FUNC_ENABLE_SERIAL, COMMAND_SERIAL_ENABLE, "Enabing serial communication"),
  initCommand(FUNC_DISABLE_SERIAL, COMMAND_SERIAL_DISABLE, "Disabling serial communication"),
  initCommand(FUNC_ENABLE_PARALLEL, COMMAND_PARALLEL_ENABLE, "Enabling parallel communication"),
  initCommand(FUNC_DISABLE_PARALLEL, COMMAND_PARALLEL_DISABLE, "Disabling parallel communication"),
  initCommand(FUNC_GET_FREQUENCY, COMMAND_FREQUENCY_GET, "Fetching sampling frequency"),
  initCommand(FUNC_SET_FREQUENCY, COMMAND_FREQUENCY_SET, "Setting sampling frequency"),
  initCommand(FUNC_ENABLE_SD, COMMAND_SD_ENABLE, "Enabling SD card"),
  initCommand(FUNC_DISABLE_SD, COMMAND_SD_DISABLE, "Disabling SD card")
};

SERCOMM SerialCommander(commands, (sizeof(commands)/sizeof(commands[0])));

//////////////////////////////////////////
/////    INTERUPT SERVICE ROUTINES   /////
//////////////////////////////////////////

void ISR_startRecording(){
  recording = true;
  TIMSK1 |= (1 << OCIE1A);                                                    // Enable the Timer1 compare interrupt A
  StartTime = millis();
  digitalWrite(LED_BUILTIN, HIGH);
}
void ISR_stopRecording(){
  recording = false;
  digitalWrite(LED_BUILTIN, LOW);
  TIMSK1 &= B11011000;                                                        // Disable Timer1

  // dataFile.close();
}

ISR(TIMER1_COMPA_vect){
  // if (SAMPLE_WINDOW)  VEC[ERREG_ARD] |= (1 << SAMPLING_RATE_LOW);
  SAMPLE_WINDOW = true;
}
ISR(ADC_vect){
  switch(ADC_MUX_SELECT){
    case 0x00:
      packt.Vref = ADC;
      analogReference(DEFAULT);					                                      // Choose the default reference
      break;
    case 0x01:
      packt.SteeringWheel = ADC;
      break;
    case 0x02:
      packt.BrakePressure = ADC;
      break;
    case 0x03:
      packt.ThrottlePositionSensor = ADC;
      break;
  } 
 
  // increment ADC_MUX_SELECT from 0x00 to 0x03 looping back when done
  ADC_MUX_SELECT = (ADC_MUX_SELECT + 1) & 0x03;

  ADMUX = (ADMUX & 0xF8) | ADC_MUX_SELECT;                                    // Set the MUX to the next ADC pin
  if (ADC_MUX_SELECT != 0x00){ 
    ADCSRA |= (1 << ADSC);                                                    // Start the conversion
  }
}
