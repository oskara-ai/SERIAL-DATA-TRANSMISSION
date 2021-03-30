///////////////////////////////////////////////////////////////////////////////////
//
//
// ArduinoUno-kortin suorat käyttöfunktiot
//
//
#include <string.h>
#include "compiler.h"
#include <avr/io.h>   
#include <avr/pgmspace.h>
#include "arduino2560.h"

#define configCPU_CLOCK_HZ	( ( uint32_t ) 16000000 )			// This F_CPU variable set by Eclipse environment

void Delay (unsigned long a) { while (--a!=0); }
	
	
/*
void InitUART()
{

//  use usart ports
PORTD=0x00;
DDRD=0x00;
DDRD=DDRD | 0b00000010; // TX port is output port

// USART0 initialization

//CLKPR |= (1<< CLKPS2); //16 pre-scaler
UCSR1A &= ~(1 << U2X0);  // single speed
UCSR1B |=  (1<< RXEN0) | (1<<TXEN0 );  // receiver on + transmitter on
UCSR1C |=  (1<< UCSZ01) | (1<< UCSZ00); // asynchronous + no parity + 8 data bits
UBRR1H=0x00;
UBRR1L = 103; // BAUD 9600
}*/

//------------------------------------------------------------------------
//	 sets the parameters for the UART port, BAUD rate and the parity 
//------------------------------------------------------------------------

//----------------------------------------------- freertos-------------------------------------------------------------
xComPortHandle SetParameters( eCOMPort ePort, uint32_t ulWantedBaud, eCOMParity parity)
{
	xComPortHandle newComPort;
	
	switch (parity)
	{
		// in case none
		case NoParity:
		UCSR0C &=  ~(1<< UPM00) | ~(1<< UPM01); // parity disabled
		break;
		
		// in case even
		case EvenParity:
		UCSR0C &= ~(1<< UPM00); // clear byte at UPM00
		UCSR0C |=  (1<< UPM01); // parity enabled, even parity
		break;
		
		// in case odd
		case OddParity:
		UCSR0C |=  (1<< UPM00) | (1<< UPM01); // parity enabled, odd parity
		break;
		
		// in case something else
		default:
		// do nothing
		break;
	}
	//--------------------------- USART 1 parity
	if ((parity == NoParity) && (newComPort.usart == USART1))
	{
		UCSR1C &=  ~(1<< UPM10) | ~(1<< UPM11); // parity disabled
	}
	else if ((parity == EvenParity)&& (newComPort.usart == USART1))
	{
		UCSR1C &= ~(1<< UPM10); // clear byte at UPM10
		UCSR1C |=  (1<< UPM11); // parity enabled, even parity
	}
	else if ((parity == OddParity) && (newComPort.usart == USART1))
	{
		UCSR1C |=  (1<< UPM10) | (1<< UPM11); // parity enabled, odd parity
	}
	//--------------------------- USART 2 parity
	if ((parity == NoParity) && (newComPort.usart == USART2))
	{
				UCSR2C &=  ~(1<< UPM20) | ~(1<< UPM21); // parity disabled
	}
	else if ((parity == EvenParity)&& (newComPort.usart == USART2))
	{
				UCSR2C &= ~(1<< UPM20); // clear byte at UPM20
				UCSR2C |=  (1<< UPM21); // parity enabled, even parity
	}
	else if ((parity == OddParity) && (newComPort.usart == USART2))
	{
		UCSR2C |=  (1<< UPM20) | (1<< UPM21); // parity enabled, odd parity
	}
	 
	//---------------------------- USART 3 parity
	if ((parity == NoParity) && (newComPort.usart == USART3))
	{
		UCSR3C &=  ~(1<< UPM30) | ~(1<< UPM31); // parity disabled
	}
	else if ((parity == EvenParity)&& (newComPort.usart == USART3))
	{
		UCSR3C &= ~(1<< UPM30); // clear byte at UPM30
		UCSR3C |=  (1<< UPM31); // parity enabled, even parity
	}
	else if ((parity == OddParity) && (newComPort.usart == USART3))
	{
		UCSR3C |=  (1<< UPM30) | (1<< UPM31); // parity enabled, odd parity
	}
	//---------------------------- -----------------------------------------
	newComPort.usart = ePort; // containing eCOMPort
	newComPort.serialWorkBufferInUse = VACANT;  // clear the occupation flag.
	newComPort.baudRate = ulWantedBaud; // containing the desired baud rate.

	switch (newComPort.usart)
	{
		/*
		 * Calculate the baud rate register value from the equation in the data sheet. */

		/* As the 16MHz Arduino boards have bad karma for serial port, we're using the 2x clock U2X0 */
		// for Arduino at 16MHz; above data sheet calculation is wrong. Need below from <util/setbaud.h>
		// This provides correct rounding truncation to get closest to correct speed.
		// Normal mode gives 3.7% error, which is too much. Use 2x mode gives 2.1% error.

		// Or, use 22.1184 MHz over clock which gives 0.00% error, for all rates.
		// Or Goldilocks Analogue uses 24.576MHz which gives 0.00% error for 9600, 19200, 38400, and 76800 baud.

	case USART1:
#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__)
		UBRR1 = (uint16_t)((configCPU_CLOCK_HZ + ulWantedBaud * 8UL) / (ulWantedBaud * 16UL) - 1); // for 1x mode, using 16 bit avr-gcc capability.
		UCSR1A = 0x00; // 1x mode.
		UCSR1B = ( _BV(RXCIE1) | _BV(RXEN1) | _BV(TXEN1));
		UCSR1C = ( _BV(USBS1) | _BV(UCSZ11) | _BV(UCSZ10) );
#else
		UBRR1 = (uint16_t)((configCPU_CLOCK_HZ + ulWantedBaud * 4UL) / (ulWantedBaud * 8UL) - 1);  // for 2x mode, using 16 bit avr-gcc capability.
		UCSR1A = _BV(U2X0); // 2x mode.		// 2x speed mode bit
		UCSR1B = ( _BV(RXCIE1) | _BV(RXEN1) | _BV(TXEN1));
		UCSR1C = ( _BV(USBS1) | _BV(UCSZ11) | _BV(UCSZ10) );
#endif
#endif
		break;

	case USART2:
#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		UBRR2 = (uint16_t)((configCPU_CLOCK_HZ + ulWantedBaud * 4UL) / (ulWantedBaud * 8UL) - 1);  // for 2x mode, using 16 bit avr-gcc capability.
		UCSR2A = _BV(U2X0); // 2x mode.		// 2x speed mode bit
		UCSR2B = ( _BV(RXCIE2) | _BV(RXEN2) | _BV(TXEN2));
		UCSR2C = ( _BV(USBS2) | _BV(UCSZ21) | _BV(UCSZ20) );
#endif
		break;

	case USART3:
#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		UBRR3 = (uint16_t)((configCPU_CLOCK_HZ + ulWantedBaud * 4UL) / (ulWantedBaud * 8UL) - 1);  // for 2x mode, using 16 bit avr-gcc capability.
		UCSR3A = _BV(U2X0); // 2x mode.		// 2x speed mode bit
		UCSR3B = ( _BV(RXCIE3) | _BV(RXEN3) | _BV(TXEN3));
		UCSR3C = ( _BV(USBS3) | _BV(UCSZ31) | _BV(UCSZ30) );
#endif
		break;

	default:
		break;
	}

	return newComPort;
}
//----------------------------------------------- freertos-------------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////
//  Serial data transmitting
//
//---------------------------------------------------------------------------
//------------------------------------------------------------------------
//	 returns the next character received from the serial port
//------------------------------------------------------------------------

//------------------------------------------------------------------------
//	 copies the supplied character to the serial port
//------------------------------------------------------------------------
void PutByte( unsigned char ch, unsigned nComPort )
{
	if ( nComPort == 1 )
	{
		while ((UCSR1A & (1<<UDRE1))==0);// wait until the transmitter buffer empty
		UDR1 = ch; //
	}
}

//------------------------------------------------------------------------
//	 copies the supplied character to certain UART port
//------------------------------------------------------------------------
void SendByte(eCOMPort port, unsigned char cData)
{
	switch(port)
	{		
		case USART1:
			while ((UCSR1A & (1<<UDRE1))==0); // wait until the transmitter buffer empty
			UDR1 = cData; 
			break;
			
		case USART2:
			while ((UCSR2A & (1<<UDRE2))==0); // wait until the transmitter buffer empty
			UDR2 = cData;
			break;
			
		case USART3:
			while ((UCSR3A & (1<<UDRE3))==0); // wait until the transmitter buffer empty
			UDR3 = cData;
			break;
			
		default:
			// do nothing
			break;
	}
}

//------------------------------------------------------------------------
//	 returns the next character received from certain UART port
//------------------------------------------------------------------------
unsigned char GetByte(eCOMPort port, unsigned char *pData)
{
	switch(port)
	{
		case USART1:
			while ((UCSR1A & (1<<RXC1))==0); //// wait until there is a character in the receiver
			return UDR1;
			break;
		case USART2:
			while ((UCSR2A & (1<<RXC2))==0); //// wait until there is a character in the receiver
			return UDR2;
			break;
		case USART3:
			while ((UCSR3A & (1<<RXC3))==0); //// wait until there is a character in the receiver
			return UDR3;
			break;
		default:
			return 0;
			break;
	}
}