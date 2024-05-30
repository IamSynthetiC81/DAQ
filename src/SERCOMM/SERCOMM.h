#ifndef SERCOMM_H
#define SERCOMM_H

#include <string.h>

#define ERROR_REG_REPORT 1
#if ERROR_REG_REPORT
  #include "../ErrorRegister/ErrorRegister.h"
#endif

typedef struct command{
  void (*function)(const int argc, char *argv[]);
  char command[40];
  char message[64];
  int argc;
  char *argv[10];
}command_t;

command_t initCommand(void (*func)(const int argc, char *argv[]), char command[], char message[]);

class SERCOMM {
  public:
    SERCOMM(command_t commands[], size_t commandsSize);

    #if ERROR_REG_REPORT
      SERCOMM(command_t commands[], size_t commandsSize, ErrorRegister *ERREG);
    #endif
    
    command_t handleCommand(const char message[], size_t len);
  private:
    size_t commandsSize;

  #if ERROR_REG_REPORT
    ErrorRegister *ERREG;
  #endif

    command_t *commands;
};
#endif