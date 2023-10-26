#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include "feather_serial.h"
#include "hall_sensor.h"
#include<stdlib.h>


// include your includes here



int main()
{
    // initialize all parameters
    //averagevoltage = 1.513044 
    char charBuffer[4];
    char arg;
    float voltage_0 = 1.513044; //from Q1.7 voltage_0 is the quiescent voltage!
    float voltage = 0;
    float averagevoltage = 0;
    float magneticField = 0;
    int value = 0;
    int distance = 40;

    // initialize the serial port on the port /dev/ttyUSB0, with baud rate 115200
    int fd = serialport_init( "/dev/ttyUSB0", 115200);
    //printf("type in a, b or c \n");
    //scanf("%c",&arg); //choose between arg == a || b || c

    serialport_write(fd, "x");
    serialport_write(fd, "y");
    //serialport_write(fd, "y");

    sleep(12);

    serialport_write(fd, "z");
    for(int i=0; i < 14; i++)
    {

        // write to the serial port to get a value (or to move the magnet according to the .ino file)
        int n = serialport_write(fd, "b"); //This line of code sends a data stored in the memory location pointed to by the address of variable 'arg' over the serial port using the file descriptor 'fd
        
/**
        // Let the user know if you were able to write to the port
        if (n == 0  )  printf("successful \n");
        else if (n == -1 )  printf("Unable to write the port \n");
**/

        // Read the sensor value from the serial port into the buffer 
        int w = serialport_read(fd, charBuffer, 4, 1000); 
        value = atoi(charBuffer);       //converting charBuffer to int with atoi

    /**
        // Let the user know if you were able to read from the port
        if (w == -1  )  printf("failed to read from the serial port \n");
        else if (w == 0 )  printf("no data was found on the port \n");
        else if (w == 1 )  printf("successful\n");
        
        printf("TESTTESTETS  %c %c %c %c  \n", charBuffer[0], charBuffer[1], charBuffer[2],charBuffer[3]);  
    **/
        
        // Convert the sensor value to a voltage
        voltage = (value / 4095.0) * 3.3;
        //if(voltage == 0)  printf("input string cannot be converted to an integer \n");  
        //printf("%f \n", voltage);

/**
        //averagevoltage will equal to the actual quiescent voltage
        //for loop is set to 20
        averagevoltage += (voltage / 20); //calculating the average voltagevalue for voltage_0
**/

        

        //print out quiescent voltage to get the value - comment out after use
        //printf("%f \n", averagevoltage);   


        //print for Q3 task 5
        

        // Convert the voltage value to magnetic field
        magneticField =  hall_sensor_get_field(voltage, voltage_0);
        // Print the magnetic field (and the position) to the terminal (or .txt file)
        //printf("The magnetic field strenght is: %f [mT)]\n", magneticField);

        printf("%d  %f \n", distance, magneticField);
        
        serialport_write(fd, "w");
        distance -= 2;
        sleep(1);
    }
    // Close the serial port
    serialport_close(fd);

	return 0;

}
