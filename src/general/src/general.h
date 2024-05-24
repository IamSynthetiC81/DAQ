#include <stdio.h>
#include <Arduino.h>

#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1

#define assert(condition) if (!(condition));
#define assert_msg(condition, message) if (!(condition));

// extern unsigned int _LINE;
// extern char _FILE[256];