#ifndef SERCOMM_H
#define SERCOMM_H

#include "../ErrorRegister/ErrorRegister.h"
#include "../general/general.h"
#include <string.h>

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
    SERCOMM(command_t commands[], size_t commandsSize, ErrorRegister *ERREG = nullptr);
    // void handler(const char* message, size_t len);
    command_t handleCommand(const char* message, size_t len);
  private:
    size_t commandsSize;

    ErrorRegister *ERREG;

    command_t *commands;
};
#endif