#include "SERCOMM.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>

#define MAX_ARGC 10

SERCOMM::SERCOMM(command_t commands[], size_t commandsSize) {
    assert(commands != NULL);
    assert(commandsSize > 0);

    this->commandsSize = commandsSize;
    this->commands = new command_t[this->commandsSize];

    // Copy contents of the commands array
    for (size_t i = 0; i < this->commandsSize; ++i) {
        this->commands[i] = commands[i];
    }

    this->ERREG = nullptr;
}

#if ERROR_REG_REPORT
SERCOMM::SERCOMM(command_t commands[], size_t commandsSize, ErrorRegister *ERREG = nullptr) {
    assert(commands != NULL);
    assert(commandsSize > 0);

    this->commandsSize = commandsSize;
    this->commands = new command_t[this->commandsSize];

    // Copy contents of the commands array
    for (size_t i = 0; i < this->commandsSize; ++i) {
        this->commands[i] = commands[i];
    }

    this->ERREG = ERREG;
}
#endif

SERCOMM::~SERCOMM() {
    delete[] this->commands;
    this->commands = nullptr;
}

command_t SERCOMM::handleCommand(char message[], size_t len) {
    // Ensure _message is null-terminated
    char _message[len + 1];
    strncpy(_message, message, len);
    _message[len] = '\0';  // Null-terminate the copied message

    command_t c;
    c.function = NULL;
    c.command[0] = '\0';
    c.message[0] = '\0';
    c.argc = 0;
    c.argv = (char**)malloc(MAX_ARGC * sizeof(char*));  // Allocate memory for argv array
    if (c.argv == NULL) {
        // Handle memory allocation failure
        return c;
    }
    for (int i = 0; i < MAX_ARGC; ++i) {
        c.argv[i] = NULL;
    }

    if (len == 0) return c;

    /* get the first token */
    char* token = strtok(_message, " ");

    for (size_t i = 0; i < this->commandsSize; i++) {
        if (strcmp(this->commands[i].command, token) == 0) {
            c.function = this->commands[i].function;
            strcpy(c.command, this->commands[i].command);
            strcpy(c.message, this->commands[i].message);
            break;
        }
    }

    /* walk through other tokens */
    while ((token = strtok(NULL, " ")) != NULL) {
        if (c.argc < MAX_ARGC) {
            c.argv[c.argc] = (char*)malloc(strlen(token) + 1);
            if (c.argv[c.argc] != NULL) {
                strcpy(c.argv[c.argc], token);
                c.argc++;
            } else {
                // Handle memory allocation failure
                break;
            }
        } else {
            // Handle too many arguments case
            break;
        }
    }

    memset(_message, '\0', len);

    return c;
}

void SERCOMM::cleanupCommand(command_t &c) {
  for (int i = 0; i < c.argc; ++i) {
    if (c.argv[i] != NULL) {
      free(c.argv[i]);
      c.argv[i] = NULL;
    }
  }
  free(c.argv);
  c.argv = NULL;
  c.argc = 0;
}

command_t initCommand(void (*func)(const int argc, char *argv[]), char command[], char message[]) {
  assert(func != NULL);
  assert(command != NULL);
  assert(message != NULL);

  command_t c;
  c.function = func;
  strcpy(c.command, command);
  strcpy(c.message, message);
  return c;
}