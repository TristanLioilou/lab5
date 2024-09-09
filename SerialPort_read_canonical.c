/**
 * @file    SerialPort_read.c
 * 
 * @brief Serial Port Programming in C (Serial Port Read)  
 * Non Cannonical mode   
 * Sellecting the Serial port Number on Linux   
 * /dev/ttyUSBx - when using USB to Serial Converter, where x can be 0,1,2...etc 
 * /dev/ttySx   - for PC hardware based Serial ports, where x can be 0,1,2...etc  
 * termios structure -  /usr/include/asm-generic/termbits.h  
 * use "man termios" to get more info about  termios structure
 * @author  Kevin Cotton
 * @date    2024-08-02
 */	
#define _GNU_SOURCE

#include <stdio.h>
#include <fcntl.h>   // File Control Definitions
#include <termios.h> // POSIX Terminal Control Definitions 
#include <unistd.h>  // UNIX Standard Definitions 
#include <errno.h>   // ERROR Number Definitions

// device port série à utiliser 
//const char *portTTY = "/dev/ttyGS0";
//const char *portTTY = "/dev/ttyS0";
const char *portTTY = "/dev/ttyS1";
//const char *portTTY = "/dev/ttyS2";
//const char *portTTY = "/dev/ttyS3";
//const char *portTTY = "/dev/ttyS4";
//const char *portTTY = "/dev/ttyS5";
//const char *portTTY = "/dev/ttyUSB0"; // ttyUSB0 is the FT232 based USB2SERIAL Converter

void main(void)
	{
	int fd; // File Descriptor
	
	printf("\n Lecture Port Serie");

	// Opening the Serial Port 
	fd = open(portTTY, O_RDWR | O_NOCTTY);  
							// O_RDWR   - Read/Write access to serial port 
							// O_NOCTTY - No terminal will control the process
							// Open in blocking mode,read will wait 
	if(fd == -1) // Error Checking
		printf("\n Erreur! ouverture de %s ", portTTY);
	else
		printf("\n Ouverture de %s reussit ", portTTY);

	// Setting the Attributes of the serial port using termios structure 
	struct termios SerialPortSettings;	// Create the structure 
	tcgetattr(fd, &SerialPortSettings);	// Get the current attributes of the Serial port 
	// Setting the Baud rate
	cfsetispeed(&SerialPortSettings, B115200); // Set Read Speed  
	cfsetospeed(&SerialPortSettings, B115200); // Set Write Speed  
	// 8N1 Mode 
	SerialPortSettings.c_cflag &= ~PARENB;   // Disables the Parity Enable bit(PARENB),So No Parity 
	SerialPortSettings.c_cflag &= ~CSTOPB;   // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
	SerialPortSettings.c_cflag &= ~CSIZE;	 // Clears the mask for setting the data size 
	SerialPortSettings.c_cflag |=  CS8;      // Set the data bits = 8  
	SerialPortSettings.c_cflag &= ~CRTSCTS;       // No Hardware flow Control
	SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver, Ignore Modem Control lines
	
	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable XON/XOFF flow control both i/p and o/p


    SerialPortSettings.c_lflag |= (ICANON | ECHO); // Activer ICANON et ECHO
	// Non Cannonical mode, Disable echo, Disable signal  
                                                

	SerialPortSettings.c_oflag &= ~OPOST;	// No Output Processing

	// Setting Time outs 
	SerialPortSettings.c_cc[VMIN] = 0; // Read at least X character(s) 
	SerialPortSettings.c_cc[VTIME] = 100; // Wait 10sec (0 for indefinetly) 

	if((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0) // Set the attributes to the termios structure
		printf("\n  Erreur! configuration des attributs du port serie");
		
	// Read data from serial port 
	tcflush(fd, TCIFLUSH);  // Discards old data in the rx buffer
	char read_buffer[32];   // Buffer to store the data received 
	int  bytes_read = 0;    // Number of bytes read by the read() system call 
	int i = 0;

	bytes_read = read(fd, &read_buffer, 32); // Read the data 
		
	printf(" Bytes Recus : %d --> ", bytes_read); // Print the number of bytes read
	for(i=0; i<bytes_read; i++)	 // printing only the received characters
		printf("%c", read_buffer[i]);

	printf("\n");

	close(fd); // Close the serial port

	}
