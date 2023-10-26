//B4: Emre Eryilmaz, David Gögle, Ved Varshney
// lab00, lab procedure, tasks 1 & 3

// source file

/* 
 * this file contains the implementation of the functions relevant for
 * lab00.
 * The declarations of the functions are in the corresponding header 
 * file (functions.h).
*/

// the header file needs to be included
#include "functions.h"

/* task 1
 * The function print_bits() accepts an input of type uint16_t 
 * (arg_word) and has no return value (void).
 * The function simply writes the binary and hexadecimal number of the
 * input to the terminal.
*/

//return binary number of a 16 bit hex number with always 4 bits together
void print_bits(uint16_t arg_word)
{
	int count = 0; //used to space the binary number after counting up to 3
	int binarray[16]; //declaration of an array called binarray

	//fill the array with corresponding bits
	for(int i = 0; arg_word > 0; i++)
	{
		binarray[i] = arg_word % 2;
		arg_word = arg_word / 2;
	}

	//zero padding: Fill the array with zeros after the most significant 1
	for(int k = 0; k < 16; k++){
		if (binarray[k] != 1){
			binarray[k] = 0;
		}
	}

	//print out the array and make a space after 4 bits
	for (int j = 15; j >= 0; j--)
	{
		printf( "%d", binarray[j]);
		count++;
		if (count == 4){
			printf(" ");
			count = 0;
		}
	}
	printf("\n");}

/* task 3
 * The function bit_merge() accepts two uint16_t as inputs (lsb and msb) 
 * and combines them to a uint32_t number by merging them.
 * The return value is a uint32_t number.
 */
 uint32_t bit_merge(uint16_t lsb, uint16_t msb)
 {
 	uint32_t msb_new = msb; //declaration of a new variable of size 32 bits

	return msb_new<<16|lsb;
 }
