#include "SERCOMM.h"
#include <assert.h>
#include <stdlib.h>

SERCOMM::SERCOMM(command_t commands[], size_t commandsSize){
  assert(commands != NULL);
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


#if ERROR_REG_REPORT
SERCOMM::SERCOMM(command_t commands[], size_t commandsSize, ErrorRegister *ERREG = nullptr){
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

command_t SERCOMM::handleCommand(const char message[], size_t len){
  assert(message != NULL);

  char _message[len];
  strncpy(_message, message, len);

  command_t c;
  c.function = NULL;
  c.command[0] = '\0';
  c.message[0] = '\0';
  c.argc = 0;
  *c.argv = nullptr;

  if (len == 0) return c;

  for(unsigned int i = 0; i < commandsSize ; i++){                                  // Go through the commands
    if (strncmp(_message, commands[i].command, strlen(commands[i].command)) == 0){  // Check if the command is in the buffer
      char *argv[10];                                                               //    Create an array of arguments
      // argv = nullptr;                                                               //    Set the first argument to NULL  
      memset(argv, '\0', sizeof(*argv));                                            //    Clear the arguments
      int argc = 0;                                                                 //    Create a counter for the arguments
      if (_message[strlen(commands[i].command)] == ' '){                            //    Check if there is a space after the command
        char *token = strtok(_message, " ");                                        //      Tokenize the buffer
        while (token != NULL){                                                      //      While there are tokens
          argv[argc] = (char *)malloc(strlen(token) + 1);
          argv[argc] = token;                                                       //        Add the token to the arguments
          argc++;                                                                   //        Increment the argument counter
          token = strtok(NULL, " ");                                                //        Get the next token
        }
      }     
      c.function = commands[i].function;                                            //    Set the function
      strcpy(c.command, commands[i].command);                                       //    Set the command
      strcpy(c.message, commands[i].message);                                       //    Set the message
      c.argc = argc;                                                                //    Set the argument count
      for (int j = 0; j < argc; j++)                                                //    Go through the arguments
        c.argv[j] = argv[j];                                                        //      Set the arguments
    }
  }
  return c;                                                                         // Return the command
}

command_t initCommand(void (*func)(const int argc, char *argv[]), char command[], char message[]){
  assert(func != NULL);
  assert(command != NULL);
  assert(message != NULL);
  
  command_t c;
  c.function = func;
  strcpy(c.command, command);
  strcpy(c.message, message);
  return c;
}
