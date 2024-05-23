#ifndef SERCOMM_H
#define SERCOMM_H

#include "../../ErrorRegister/src/ErrorRegister.h"
#include "../../general/src/general.h"
#include <string.h>



typedef struct command{
  void (*function)(const int argc, char *argv[]);
  char command[16];
  char message[128];
}command_t;

command_t initCommand(void (*func)(const int argc, char *argv[]), char command[], char message[]);

class SERCOMM {
  public:
    SERCOMM(command_t commands[], size_t commandsSize, size_t BufferSize, ErrorRegister *ERREG = nullptr);
    void handler(const char* message, size_t len);
    void setCommands(command_t commands[], size_t commandsSize);
    size_t getBufferSize();
  private:
    char* buffer = new char[BufferSize];
    size_t BufferSize = 20;
    size_t bufferIndex = 0;

    size_t commandsSize;

    ErrorRegister *ERREG;

    command_t *commands;
};
#endif