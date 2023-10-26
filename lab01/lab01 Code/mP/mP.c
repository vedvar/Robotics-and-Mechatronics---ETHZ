//David Gögl, Ved Varshney, Emre Eryilmaz
#include <stdio.h>
#include <stdint.h>
#include "feather_serial.h"



// ***** print_bits() can be implemented here ***** //

void print_bits(uint8_t arg_word)
{
	int count = 0; //used to space the binary number after counting up to 3
	int binarray[8];

	//fill the array with corresponding bits
	for (int i = 0; arg_word > 0; i++)
	{
		binarray[i] = arg_word % 2;
		arg_word = arg_word / 2;
	}

	//zero padding: Fill the array with zeros after the most significant 1
	for (int k = 0; k < 8; k++)
	{
		if (binarray[k] != 1) {
			binarray[k] = 0;
		}
	}
	//print out the array and make a space after 4 bits
	for (int j = 7; j >= 0; j--)
	{
		printf( "%d", binarray[j]);
		count++;
		if (count == 4) {
			printf(" ");
			count = 0;
		}
	}
	printf("\n");
}

// *********************************************** //

int32_t main()
{
	int n;
	// initialization of serial communication, port and baud rate are specified
	int fd = serialport_init( "/dev/ttyUSB0", 115200);

	//Cast the value 245 to a 8-bit unsigned integer.
	uint8_t arg = (uint8_t)245;

	while (1) {

		// ***** your code goes here ***** //

		printf("Input number #1: \n");
		int num1;
		scanf("%d", &num1);                      //The & before num1 is so that scanf knows at which adress to store the input
		printf("Your input was: %d \n", num1);

		printf("input number #2: \n");
		int num2;
		scanf("%d", &num2);
		printf("Your input was: %d \n", num2);

		//input sequence to terminate the while(1) loop
		if (num1 == 0000 || num2 == 0000) {
			break;
		}

		print_bits(num1 + num2);
		arg = num1 + num2;

		// ******************************* //
		// send arg via serial communication to the mC
		// type casting is again needed to match type
		n = serialport_writebyte(fd, ((char*)&arg));    //This line of code sends a byte of data stored in the memory location pointed to by the address of variable 'arg' over the serial port using the file descriptor 'fd
		if (n == -1 )
			printf("Unable to write the port \n");
	}

	// close serial communication
	serialport_close(fd);

	return 0;

}
