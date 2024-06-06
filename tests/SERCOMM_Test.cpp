// #define FAIL_ON_ERROR

#include "testTools.h"

// Include the header file for the SERCOMM module
#include "../src/SERCOMM/SERCOMM.h"

void testRoutine_01(const int argc, char *argv[]){
	char str[] = "Routine wiht no args \t";
	std::cout << str;
}
void testRoutine_02(const int argc, char *argv[]){
	char str[] = "Routine with no args  \t";
	std::cout << str;
}
void testRoutine_03(const int argc, char *argv[]){
	char str[] = "Routine with args  \t";
	std::cout << str;
}

bool testRoutine(SERCOMM* uut, command_t cut, int argc, char *argv[]){
	// construct the command
	char* command = (char*)malloc(strlen(cut.command) + 1);
	strcpy(command, cut.command);

	for(int i = 0 ; i < argc; i++){
		command = (char*)realloc(command, strlen(command) + strlen(argv[i]) + 1);
		strcat(command, " ");
		strcat(command, argv[i]);
	}
	
	// Call the handleCommand function
	command_t c = uut->handleCommand(command, strlen(command));
		
	// Test the function
	c.function(argc, argv);

	assert(c.argc == argc);
		
	for(int i = 0; i < argc; i++){
		assert(strcmp(c.argv[i], argv[i]) == 0);
	}

	free(command);

	return true;
}

int main() {

	// Create an array of command objects
	command_t commands[] = {
		initCommand(&testRoutine_01, "testRoutine_01", "Test routine 01 message"),
		initCommand(&testRoutine_02, "testRoutine_02", "Test routine 02 message"),
		initCommand(&testRoutine_03, "testRoutine_03", "Test routine 03 message")
	};

	// Create an instance of the SERCOMM class
	SERCOMM sercomm(commands, 3);

	// Create an array of arguments	
	char* argv[] = {"testRoutine_03", "arg1", "arg2", "arg3"};

	std::cout << "\n\tRunning SERCOMM Test..." << std::endl;

	// Test the handleCommand function
	print_result(testRoutine(&sercomm, commands[0], 0, nullptr));
	print_result(testRoutine(&sercomm, commands[1], 0, nullptr));
	print_result(testRoutine(&sercomm, commands[2], 4, argv));
	return 0;
}