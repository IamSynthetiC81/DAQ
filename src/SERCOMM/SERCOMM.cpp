#include "SERCOMM.h"

SERCOMM::SERCOMM(command_t commands[], size_t commandsSize, ErrorRegister *ERREG = nullptr){
  assert_msg(commands != NULL, "Commands are NULL");
  assert_msg(commandsSize > 0, "Commands size is 0");

  this->commandsSize = commandsSize;
  this->commands = new command_t[this->commandsSize];
  
  // Copy contents of the commands array
  for (size_t i = 0; i < this->commandsSize; ++i) {
    this->commands[i] = commands[i];
  }
  
  this->ERREG = ERREG;
}

// void SERCOMM::handler(const char* message, size_t len){
//   if (bufferIndex + len > BufferSize){
//     memset(buffer, '\0', BufferSize);
//     return;
//   } else if (len == 0){
//     return;
//   } 

//   // copy message to the end of buffer
//   memcpy(buffer + bufferIndex, message, len);
  
//   for(int i = 0; i < commandsSize ; i++){                                    // Go through the commands
//     if (strncmp(buffer, commands[i].command, strlen(commands[i].command)) == 0){ // Check if the command is in the buffer
//       if (buffer[strlen(commands[i].command)] == ' '){                        // Check if there is a space after the command
//         char *argv[10];                                                       // Create an array of arguments
//         int argc = 0;                                                         // Create a counter for the arguments
//         char *token = strtok(buffer, " ");                                    // Tokenize the buffer
//         while (token != NULL){                                                // While there are tokens
//           argv[argc] = token;                                                 // Add the token to the arguments
//           argc++;                                                             // Increment the argument counter
//           token = strtok(NULL, " ");                                          // Get the next token
//         }
//         commands[i].function(argc, argv);                                     // Call the function with the arguments

//         memset(buffer,'\0', BufferSize);                                      // Clear the buffer
//         bufferIndex = 0;                                                      // Reset the buffer index
//         return;
//       }
//     }
//   }
// }

command_t SERCOMM::handleCommand(const char* message, size_t len){
  command_t c;
  c.function = NULL;
  c.command[0] = '\0';
  c.message[0] = '\0';

  if (len == 0) return c;

  for(int i = 0; i < commandsSize ; i++){                                           // Go through the commands
    if (strncmp(message, commands[i].command, strlen(commands[i].command)) == 0){   // Check if the command is in the buffer
      char *argv[10];                                                               //    Create an array of arguments
      memset(argv, '\0', 10);                                                       //    Clear the arguments
      int argc = 0;                                                                 //    Create a counter for the arguments
      if (message[strlen(commands[i].command)] == ' '){                             //    Check if there is a space after the command
        char *token = strtok(message, " ");                                         //      Tokenize the buffer
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
  assert_msg(func != NULL, "Function is NULL");
  assert_msg(command != NULL, "Command is NULL");
  assert_msg(message != NULL, "Message is NULL");

  assert_msg_var(strlen(command) > 64, "Command is too long" , command);
  assert_msg_var(strlen(message) > 40, "Message is too long", message);
  
  command_t c;
  c.function = func;
  strcpy(c.command, command);
  strcpy(c.message, message);
  return c;
}
