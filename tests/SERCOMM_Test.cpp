#include <iostream>
#include <cassert>

// Define color codes
#define RED     "\x1b[31m"
#define GREEN   "\x1b[32m"
#define RESET   "\x1b[0m"

// Function to print the result in color
void print_result(bool success) {
	if (success) {
			std::cout << GREEN << "[" << "PASS" << "]" << RESET << std::endl;
	} else {
			std::cout << RED << "[" << "FAIL" << "]" << RESET << std::endl;
	}
}

// Include the header file for the SERCOMM module
#include "../src/SERCOMM/SERCOMM.h"
// #include <SERCOMM.h>

void testRoutine_01(const int argc, char *argv[]){
	char str[] = "Test routine 01 is running!\r\n";
	std::cout << str;
}

void testRoutine_02(const int argc, char *argv[]){
	char str[] = "Test routine 02 is running!\r\n";
	std::cout << str;
}

void testRoutine_03(const int argc, char *argv[]){
	char str[] = "Test routine 03 is running!\r\n";
}

bool testRoutine(){
	// Create a command array
	command_t commands[] = {
		initCommand(&testRoutine_01, "testRoutine_01", "Running Test routine 01"),
		initCommand(&testRoutine_02, "testRoutine_02", "Running Test routine 02"),
		initCommand(&testRoutine_03, "testRoutine_03", "Running Test routine 03")
	};	

	// Create a SERCOMM object
	SERCOMM sercomm(commands, 3);

	// Test the handleCommand function
	command_t c = sercomm.handleCommand("testRoutine_01", 14);
	assert(c.function == testRoutine_01);
	assert(strcmp(c.command, "testRoutine_01") == 0);
	assert(strcmp(c.message, "Running Test routine 01") == 0);
	assert(c.argc == 0);
	assert(*c.argv == nullptr);

	std::cout << "Test 1:\t\t";
	print_result(true);

	// run function
	c.function(c.argc, c.argv);


	c = sercomm.handleCommand("testRoutine_02", 14);
	assert(c.function == testRoutine_02);
	assert(strcmp(c.command, "testRoutine_02") == 0);
	assert(strcmp(c.message, "Running Test routine 02") == 0);
	assert(c.argc == 0);
	assert(c.argv[0] == NULL);

	std::cout << "Test 2:\t\t";
	print_result(true);

	// run function
	c.function(c.argc, c.argv);

	c = sercomm.handleCommand("testRoutine_03 arg1 arg2 arg3 \n", 27);

/*
	printf("Command: %s\n", c.command);
	printf("Message: %s\n", c.message);
	printf("Argc: %d\n", c.argc);
	printf("Argv[0]: %s\n", c.argv[0]);
	printf("Argv[1]: %s\n", c.argv[1]);
	printf("Argv[2]: %s\n", c.argv[2]);
	printf("Argv[3]: %s\n", c.argv[3]);
*/

/*
	assert(c.function == testRoutine_03);
	assert(strcmp(c.command, "testRoutine_03") == 0);
	assert(strcmp(c.message, "Running Test routine 03") == 0);
	assert(c.argc == 4);
	assert(strcmp(c.argv[0], "testRoutine_03") == 0);
	assert(strcmp(c.argv[1], "arg1") == 0);
	assert(strcmp(c.argv[2], "arg2") == 0);
	assert(strcmp(c.argv[3], "arg3") == 0);

	std::cout << "Test 3:\t\t";
	print_result(true);
*/
	// run function
	c.function(c.argc, c.argv);

	return true;
}

int main() {
	
	std::cout << "\n\tRunning SERCOMM Test..." << std::endl;
	testRoutine();

	return 0;
}