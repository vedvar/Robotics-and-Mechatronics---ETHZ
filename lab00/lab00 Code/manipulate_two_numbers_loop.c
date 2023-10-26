//B4: Emre Eryilmaz, David Gögle, Ved Varshney
// lab00, lab procesure, task 4

// source file for the program manipulate_two_numbers

/*
 * This file is used to generate an executable program that reads two
 * numbers from the terminal and output the merged result in 
 * hexadecimal, and the sum in hexadecimal and binary format.
*/

// again, the header of our function library is already included 
#include "functions.h"

// main
int main(int argc, char *argv[])
{
	while(1)
	{
		int number_1 = 0;
		int number_2 = 0;
		int stop_number1 = 0x0000;
		int stop_number2 = 0x1111;

		printf("Enter First Hexadecimal Number\n");
		scanf("%x", &number_1);

		printf("Enter Second Hexadecimal Number\n");
		scanf("%x", &number_2);

		if(number_1 == stop_number1 && number_2 == stop_number2 
			|| number_1 == stop_number2 && number_2 == stop_number1)
		{
			printf("stopping program\n");
			break;
		} 

		printf("merging 0x%x and  0x%x results in 0x%08x\n", number_1, number_2, bit_merge(number_1, number_2));

		printf("the sum in hex is 0x%x, bin: ", number_1 + number_2 );
		print_bits(number_1 + number_2);
	}
	return 0;
}