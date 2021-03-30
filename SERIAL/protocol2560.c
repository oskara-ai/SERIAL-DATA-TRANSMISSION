

// this many SYNs before message at least
#define COMM_SYN_CHAR_COUNT 5

#include "compiler.h"
#include <avr/io.h>        
#include <stdlib.h>             
#include "arduino2560.h"
#include "protocol.h"



/*******************************************************************
**
** Output:   */ int   /*
**                                 
** Function: */ SendMsg( unsigned char *pData, /*  leaving data
**           */          unsigned short cBytes )/*  message length
**
** Description: sends message in the envelope to other end
**              
**
** Remarks:   
**
********************************************************************/
{
	int i;
    PutByte(STX,1);
    PutByte(0,1);
    PutByte(cBytes,1);

    // send the data
    for ( i = 0; i < cBytes; i++ ) 
		  PutByte(pData[i],1);
    
    //calculate and send CheckSum
	unsigned char x = 0;
    for( i= 0; i < cBytes ; i++ )
	    x += pData[ i ];
 	PutByte(x,1);
	 
	// send end of the message
    PutByte(ETX,1);

    return 1;
}

/*******************************************************************
**
** Output:   */ int  /*  
**                                 
** Function: */ ReceiveMsg( unsigned char *pData, /*  coming data
**           */             unsigned short *pcBytes )/*  message length
**
** Description: reads message in the envelope from other end
**              
**             
** Remarks:   
**
********************************************************************/
{
   // DWORD cBytesRead = 0;
    unsigned char  chLast;
    unsigned short lenMsg;
    unsigned short cData;
    unsigned char  chkSum;
    int  cSYN = 0;


    // sychronize to the STX byte
    //
    // sometimes the synchronizing can happend at wrong place ( into data bytes )
    // if this is a problem , the protocol should be developed so that
    // we use series of SYN bytes for this purpose
    //
    // all commands have SYN characters before the message
       while ( cSYN < COMM_SYN_CHAR_COUNT )
      {
         chLast= GetByteOriginal(1);
         if ( chLast == SYN )
            cSYN++;
      }

    // message starts with STX character
    while ( 1 )   // wait until STX comes
    {
    		chLast = GetByteOriginal(1);
         if ( chLast != STX)
             continue;
         else
             break;
    }

    // message len
   GetByteOriginal(1); // only short messages used, discard upper byte
   lenMsg = GetByteOriginal(1);
   if ( lenMsg > COMM_MSG_LEN_MAX )
      return 0;

    // pick up data
    cData = 0;
    while (  cData < lenMsg )
    {
        chLast = GetByteOriginal(1);
        if ( cData < *pcBytes ) // space for all characters ?
           pData[ cData++ ] = chLast;
        else
           return 0;
    }

    // check sum
    chkSum = GetByteOriginal(1);

    // etx  is the last character in message
    chLast = GetByteOriginal(1);
    if ( chLast != ETX)
      return 0;

    // is check sum right ?
    {
        unsigned  short iTemp;
        unsigned  char dataSum = 0;

        dataSum  = 0;
        for( iTemp= 0; iTemp <   lenMsg; iTemp++)
            dataSum += pData[ iTemp ];

        if ( chkSum - dataSum == 0 )
        {
            *pcBytes = lenMsg; // return message len
            return 1;
        }
        else
           return 0;
    }
}


