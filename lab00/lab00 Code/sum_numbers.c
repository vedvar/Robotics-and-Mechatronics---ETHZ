//B4: Emre Eryilmaz, David Gögle, Ved Varshney
// lab00, lap procedure, task 2

// source file for the program sum_numbers

/*
 * This file is used to generate an executable program that prints the 
 * sum of the two numbers in binary and hexadecimal format.
*/

// Since we will be using our own functions, we need to add the header
// file where the functions are declared.
// Keep in mind that the header file already includes standard libraries
// so they do not to be included here.
#include "functions.h"
 
 // main function
int main(int argc, char *argv[])
{
	uint16_t a = 0x384A;
	uint16_t b = 0x1111;
	uint16_t c = a+b;

	    printf("%x\n", c);

	    print_bits(c);

	return 0;
}
