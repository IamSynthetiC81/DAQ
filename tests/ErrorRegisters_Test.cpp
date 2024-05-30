#include "testTools.h"

#include "../src/ErrorRegister/ErrorRegister.h"

ErrorRegister UUT;

bool testSetBit(){

	UUT.SETREG(0x00);

	for(int i = 0; i < 8; i++){
		if(UUT.SET(i) == false) return false;
		if(UUT.GET(i) == false) return false;
	}

	return true;
}

bool testClearBit(){

	UUT.SETREG(0xFF);

	for(int i = 0; i < 8; i++){
		if(UUT.CLEAR(i) == false) return false;
		if(UUT.GET(i) == true) return false;
	}

	return true;
}

bool testSetReg(){

	UUT.SETREG(0x00);

	for (uint8_t i = 1; i < UINT8_MAX ; i++){
		uint8_t prevVal = UUT.SETREG(i);

		if(prevVal != (i-1)) return false;
		if (UUT.GET() != i) return false;
	}

	return true;
}

bool testClearReg(){

	UUT.SETREG(0xFF);

	for (uint8_t i = 0; i < UINT8_MAX ; i++){
		UUT.CLEAR(i);
		if (UUT.GET(i)) return false;
	}

	return true;
}

bool testOutOfBounds(){

	if(UUT.SET(8) == true) return false;
	if(UUT.CLEAR(8) == true) return false;
	if(UUT.GET(8) == true) return false;

	return true;
}

int main(void){
	
	std::cout << "\nTesting Error Registers..." << std::endl;

	std::cout << "SET : \t\t\t";
	_assert(testSetBit());

	std::cout << "CLEAR : \t\t";
	_assert(testClearBit());

	std::cout << "SETREG : \t\t";
	_assert(testSetReg());

	std::cout << "CLEARREG : \t\t";
	_assert(testClearReg());

	std::cout << "OutOfBounds : \t\t";
	_assert(testOutOfBounds());

	return 0;
}