#include <stdio.h>
#include <Arduino.h>
#include <avr/wdt.h>


#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1

#define assert(condition) if (!(condition));
#define assert_msg(condition, message) if (!(condition)) Serial.print(message);
#define assert_msg_var(condition, message, var) if (!(condition)) Serial.println(String(message) + var);

#define error(message) Serial.println(message);
// #define error_set(message, errorreg, bit) Serial.println(\n\tmessage\n);
// #define error_setb(message, errorreg, bit) Serial.println(\n\tmessage\n);

#define fatal(message) Serial.println(message); fatalBlink() ;
#define log(message) Serial.println(message);
#define logln(message) Serial.println(message);

#define fatalBlink() while(1){digitalWrite(LED_BUILTIN, HIGH);delay(100);digitalWrite(LED_BUILTIN, LOW);delay(100);};

// extern unsigned int _LINE;
// extern char _FILE[256];
