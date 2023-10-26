//B4: Emre Eryilmaz, David Gögle, Ved Varshney
// lab00, lab procedure, tasks 1 & 3

// header file

/*
 * This file contains the declarations of the relevant functions.
 * The implementation of each function can be found in the source file
 * (functions.c)
*/

// some standard libraries are already inluded
#include <stdio.h>
#include <stdint.h>

/* task 1
 * The function print_bits() accepts an input of type uint16_t and has 
 * no return value (void).
 * The function simply writes the binary and hexadecimal number of the
 * input to the terminal.
*/
void print_bits(uint16_t arg_word);

/* task 3
 * The function bit_merge() accepts two uint16_t as inputs and combines
 * them to a uint32_t number by merging them.
 * The return value is a uint32_t number.
 */
 uint32_t bit_merge(uint16_t lsb, uint16_t msb);
