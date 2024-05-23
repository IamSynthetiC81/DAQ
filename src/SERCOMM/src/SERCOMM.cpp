#include "SERCOMM.h"

SERCOMM::SERCOMM(command_t commands[], size_t commandsSize, size_t BufferSize, ErrorRegister *ERREG = nullptr){
  assert_msg(commands != NULL, "Commands are NULL");
  assert_msg(commandsSize > 0, "Commands size is 0");
  assert_msg(BufferSize > 0, "Buffer size is 0");
  assert_msg(ERREG != NULL, "Error register is NULL");

  this->commandsSize = commandsSize;
  this->commands = new command_t[this->commandsSize];
  
  // Copy contents of the commands array
  for (size_t i = 0; i < this->commandsSize; ++i) {
    this->commands[i] = commands[i];
  }
  
  this->BufferSize = BufferSize;
  this->buffer = new char[this->BufferSize];

  memset(buffer, 0, this->BufferSize);

  this->ERREG = ERREG;
}

void SERCOMM::handler(const char* message, size_t len){
  if (bufferIndex + len > BufferSize){
    memset(buffer, 0, BufferSize);
    return;
  } else if (len == 0){
    return;
  } 

  // copy message to the end of buffer
  memcpy(buffer + bufferIndex, message, len);
  
  // parse each command
  for(int i = 0; i < commandsSize ; i++){                                    // Go through the commands
    if (strncmp(buffer, commands[i].command, strlen(commands[i].command)) == 0){ // Check if the command is in the buffer
      if (buffer[strlen(commands[i].command)] == ' '){                        // Check if there is a space after the command
        char *argv[10];                                                       // Create an array of arguments
        int argc = 0;                                                         // Create a counter for the arguments
        char *token = strtok(buffer, " ");                                    // Tokenize the buffer
        while (token != NULL){                                                // While there are tokens
          argv[argc] = token;                                                 // Add the token to the arguments
          argc++;                                                             // Increment the argument counter
          token = strtok(NULL, " ");                                          // Get the next token
        }
        commands[i].function(argc, argv);                                     // Call the function with the arguments

        memset(buffer, 0, BufferSize);                                        // Clear the buffer
        bufferIndex = 0;                                                      // Reset the buffer index
        return;
      }
    }
  }
}

size_t SERCOMM::getBufferSize(){
  return BufferSize;
}

command_t initCommand(void (*func)(const int argc, char *argv[]), char command[], char message[]){
  assert_msg(func != NULL, "Function is NULL");
  assert_msg(command != NULL, "Command is NULL");
  assert_msg(message != NULL, "Message is NULL");
  
  command_t c;
  c.function = func;
  strcpy(c.command, command);
  strcpy(c.message, message);
  return c;
}
