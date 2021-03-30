//-------------------------------------------------------------------------------
//
// Tämä on Arduino2560-kortille toteutettu sarjaliikenneversio, joka käyttää varmistettua protokollaa.
//
//  1. Laita ensin usb-kaapeli paikalleen.   PC:n sarjaportin parametrit 19200-8-N-1.
//  
//-------------------------------------------------------------------------------

#include "compiler.h"
#include <avr/io.h>    
#include <stdlib.h>    
#define F_CPU 16000000UL
#include <util/delay.h>

#include "arduino2560.h"  // ArduinoUno-kortin käyttö
#include "protocol.h"    // tietoliikenneprotokolla
#include "application.h" // sovelluskohtaiset vakiot ja esittelyt

/* Baud rate used by the serial port tasks. */
#define mainCOM_BAUD_RATE			(9600)
#define comBUFFER_LEN				(50)
xComPortHandle xSerialPort;

int main()
{
	unsigned char  port;
	unsigned char pData;
	
	xSerialPort = SetParameters(USART1, mainCOM_BAUD_RATE, NoParity);
	xSerialPort = SetParameters(USART2, mainCOM_BAUD_RATE, NoParity);
	
	
// forever loop
    while ( 1 )
    {
		port = GetByte(USART2, &pData);
		SendByte(USART2, port);
	}
}
