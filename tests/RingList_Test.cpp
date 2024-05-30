#include <iostream>
#include <string>
#include <cassert>
#include "../src/RingList/RingList.h"

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

void testRingList() {
    RingList<int> ring(3);
    bool success;

    // Test push
    success = ring.push(1) == 0;
    std::cout << "Push 1: \t\t";
    print_result(success);
    assert(success);

    success = ring.push(2) == 0;
    std::cout << "Push 2: \t\t";
    print_result(success);
    assert(success);

    success = ring.push(3) == 0;
    std::cout << "Push 3: \t\t";
    print_result(success);
    assert(success);

    success = ring.push(4) == 1; // Should fail because the buffer is full
    std::cout << "Push 4: \t\t";
    print_result(success);
    assert(success);

    // Test pop
    int value;
    value = ring.pop();
    success = value == 1;
    std::cout << "Pop[1]: \t\t";
    print_result(success);
    assert(success);

    value = ring.pop();
    success = value == 2;
    std::cout << "Pop[2]: \t\t";
    print_result(success);
    assert(success);

    value = ring.pop();
    success = value == 3;
    std::cout << "Pop[3]: \t\t";
    print_result(success);
    assert(success);

    value = ring.pop();
    success = ring.isEmpty(); // Should be empty after popping all elements
    std::cout << "Pop empty: \t\t";
    print_result(success);
    assert(success);

    // Test isEmpty and isFull
    success = ring.isEmpty();
    std::cout << "isEmpty: \t\t";
    print_result(success);
    assert(success);

    success = !ring.isFull();
    std::cout << "isFull: \t\t";
    print_result(success);
    assert(success);

    success = ring.push(5) == 0;
    std::cout << "push[4]: \t\t";
    print_result(success);
    assert(success);

    success = !ring.isEmpty();
    std::cout << "isEmpty: \t\t";
    print_result(success);
    assert(success);

    success = ring.push(6) == 0;
    std::cout << "Push[5]: \t\t";
    print_result(success);
    assert(success);

    success = ring.push(7) == 0;
    std::cout << "Push[6]: \t\t";
    print_result(success);
    assert(success);

    success = ring.isFull();
    std::cout << "Push[7]: \t\t";
    print_result(success);
    assert(success);

    success = ring.push(8) == 1; // Should fail because the buffer is full
    std::cout << "Push[8]: \t\t";
    print_result(success);
    assert(success);

    // Test getSize
    success = ring.getSize() == 3;
    std::cout << "getSize: \t\t";
    print_result(success);
    assert(success);
}

int main() {
    std::cout << "\n\tRunning RingList Test..." << std::endl;
    testRingList();
    return 0;
}
