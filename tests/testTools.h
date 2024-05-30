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

#define _assert(x) assert(x); print_result(x)