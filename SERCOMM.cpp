#include "SERCOMM.h"

static HardwareSerial *str;

static bool *flag = NULL;
static unsigned long *time = NULL;
static unsigned int *sfreq = NULL;

static void (*exportFunc)(void) = NULL;
static void (*ExportSerial)(void) = NULL;
static void (*ExportSD)(void) = NULL;

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


typedef union function{
  void (*Voidfunc)(void);
  void (*Setfunc)(unsigned int);
} function_t;

typedef struct command{
  function_t function;
  char command[16];
  char message[128];
}command_t;

command_t initCommand(void (*func)(void), char command[], char message[]){
  command_t c;
  c.function.Voidfunc = func;
  strcpy(c.command, command);
  strcpy(c.message, message);
  return c;
}

command_t initCommand(void (*func)(unsigned int), char command[], char message[]){
  command_t c;
  c.function.Setfunc = func;
  strcpy(c.command, command);
  strcpy(c.message, message);
  return c;
}

void FUNC_START_SAMPLING(){
  *flag = true;
  TIMSK1 |= (1 << OCIE1A);                                                    // Enable the Timer1 compare interrupt A
  *time = millis();

  // digitalWrite(LED_BUILTIN, HIGH);
  PORTB |=  (1 << PIN7);
}
void FUNC_STOP_SAMPLING(){
  *flag = false;

  // digitalWrite(LED_BUILTIN, LOW);
  PORTB &= !(1 << PIN7);
}
void FUNC_RESET(){
  asm volatile ("  jmp 0");
}
void FUNC_ENABLE_SERIAL(){
  exportFunc = ExportSerial;
}
void FUNC_DISABLE_SERIAL(){
  exportFunc = NULL;
}
void FUNC_ENABLE_PARALLEL(){
  // exportFunc = ExportParallel;
}
void FUNC_DISABLE_PARALLEL(){
  exportFunc = NULL;
}
void FUNC_GET_FREQUENCY(){
  str->print("Sampling Frequency: ");
  str->println(*sfreq);
}
void FUNC_SET_FREQUENCY(unsigned int _sfreq){
  *sfreq = _sfreq;
  str->print("Sampling Frequency set to: ");
  str->println(*sfreq);
}
void FUNC_ENABLE_SD(){
  exportFunc = ExportSD;
}
void FUNC_DISABLE_SD(){
  exportFunc = NULL;
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

/**
 * Handler for the commands. reads each char ,and like a binary tree, it goes through the commands
*/
void SERCOMM_handler(const char* message, size_t len){
  for(int i = 0; i < sizeof(commands)/sizeof(commands[0]); i++){                                    // Go through the commands
    if(strncmp(message, commands[i].command, strlen(commands[i].command)) == 0){                    // If the command is found
      if(commands[i].function.Voidfunc != NULL){                                                    // If the function is a void function
        str->println(commands[i].message);                                                          // Print the message                          
        commands[i].function.Voidfunc();                                                            // Call the function                    
      } else if(commands[i].function.Setfunc != NULL){                                              // If the function is a set function                
        str->println(commands[i].message);                                                          // Print the message                      
        commands[i].function.Setfunc(atoi(message + strlen(commands[i].command)));                  // Call the function with the argument
      }
    }
  }
}

void SERCOMM_init(Stream *stream, bool *_flag, unsigned long *_time, unsigned int *_sfreq, void (*_ExportSerial)(void), void (*_ExportSD)(void)){
  str = stream;
  flag = _flag;
  time = _time;
  sfreq = _sfreq;

  // static_assert(_ExportSerial != NULL, "ExportSerial function is not defined");
  // static_assert(_ExportSD != NULL, "ExportSD function is not defined");

  ExportSerial = _ExportSerial;
  ExportSD = _ExportSD;
}

