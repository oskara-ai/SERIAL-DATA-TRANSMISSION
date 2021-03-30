#ifndef F_CPU
#define F_CPU 16000000UL
#endif

typedef struct
{
	volatile uint16_t count;	/**< Number of bytes currently stored in the buffer. */
	volatile uint8_t * in;		/**< Current storage location in the circular buffer. */
	volatile uint8_t * out;		/**< Current retrieval location in the circular buffer. */
	uint8_t* start;				/**< Pointer to the start of the buffer's underlying storage array. */
	uint8_t* end;				/**< Pointer to the end of the buffer's underlying storage array. */
	uint16_t size;				/**< Size of the buffer's underlying storage array. */
} ringBuffer_t, * ringBufferPtr_t;

#ifdef __cplusplus
extern "C" {
	#endif

	typedef enum
	{
		USART0,
		USART1,
		USART2,
		USART3,
	} eCOMPort;
	
	typedef enum
	{
		NoParity,
		EvenParity,
		OddParity
	} eCOMParity;

	typedef enum
	{
		VACANT,
		ENGAGED
	} binary;

	typedef struct
	{
		eCOMPort usart;
		ringBuffer_t xRxedChars;
		ringBuffer_t xCharsForTx;
		uint8_t *serialWorkBuffer;		// create a working buffer pointer, to later be malloc() on the heap.
		uint16_t serialWorkBufferSize;	// size of working buffer as created on the heap.
		binary	serialWorkBufferInUse;	// flag to prevent overwriting by multiple tasks using the same USART.
		uint32_t baudRate;				// configured baud rate.
	} xComPortHandle, * xComPortHandlePtr;

#ifndef ARDUINO_2560_H
#define ARDUINO_2560_H

//! This module contains low level hardware abstraction layer for Arduino 2560 board


#define  LED_PORT  PORTB
#define  LED_DDR   DDRB
#define  LED_PIN   PINB


#define  LED_STATUS_BIT  PB7 // digital pin 13

#define  Led_init()             (LED_DDR  |=  (1<<LED_STATUS_BIT))                            
#define  Led_on()               (LED_PORT |=  (1<<LED_STATUS_BIT))
#define  Led_off()              (LED_PORT &= ~(1<<LED_STATUS_BIT))
#define  Led_toggle()           (LED_PORT  ^=  (1<<LED_STATUS_BIT))
#define  Is_led_on()            (LED_PIN  &   (1<<LED_STATUS_BIT) ? TRUE : FALSE)


// functions in Arduino2560.c
void InitUART(void);
void PutByte( unsigned char ch, unsigned nComPort );
void Delay (unsigned long a);
void SendByte(eCOMPort port, unsigned char cData);
unsigned char GetByte(eCOMPort port, unsigned char *pData);
xComPortHandle SetParameters( eCOMPort ePort, uint32_t ulWantedBaud, eCOMParity parity);

#endif  // Arduino 2560


