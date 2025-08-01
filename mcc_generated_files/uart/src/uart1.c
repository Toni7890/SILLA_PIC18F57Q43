/**
 * UART1 Generated Driver API Header File
 * 
 * @file uart1.c
 * 
 * @ingroup uart1
 * 
 * @brief This is the generated driver implementation file for the UART1 driver using the Universal Asynchronous Receiver and Transmitter (UART) module.
 *
 * @version UART1 Driver Version 3.0.8
*/

/*
� [2025] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

/**
  Section: Included Files
*/
#include "../uart1.h"

/**
  Section: Macro Declarations
*/

#define UART1_TX_BUFFER_SIZE (8U) //buffer size should be 2^n
#define UART1_TX_BUFFER_MASK (UART1_TX_BUFFER_SIZE - 1U) 

#define UART1_RX_BUFFER_SIZE (8U) //buffer size should be 2^n
#define UART1_RX_BUFFER_MASK (UART1_RX_BUFFER_SIZE - 1U)

/**
  Section: UART1 variables
*/
static volatile uint8_t uart1TxHead = 0;
static volatile uint8_t uart1TxTail = 0;
static volatile uint8_t uart1TxBuffer[UART1_TX_BUFFER_SIZE];
static volatile uint8_t uart1TxBufferRemaining;

static volatile uint8_t uart1RxHead = 0;
static volatile uint8_t uart1RxTail = 0;
static volatile uint8_t uart1RxBuffer[UART1_RX_BUFFER_SIZE];
/**
 * @misradeviation{@advisory,19.2}
 * The UART error status necessitates checking the bitfield and accessing the status within the group byte therefore the use of a union is essential.
 */
 /* cppcheck-suppress misra-c2012-19.2 */
static volatile uart1_status_t uart1RxStatusBuffer[UART1_RX_BUFFER_SIZE];
static volatile uint8_t uart1RxCount;

 /**
 * @misradeviation{@advisory,19.2}
 * The UART error status necessitates checking the bitfield and accessing the status within the group byte therefore the use of a union is essential.
 */
 /* cppcheck-suppress misra-c2012-19.2 */
static volatile uart1_status_t uart1RxLastError;

/**
  Section: UART1 APIs
*/

static void (*UART1_FramingErrorHandler)(void);
static void (*UART1_OverrunErrorHandler)(void);
static void (*UART1_ParityErrorHandler)(void);
void (*UART1_TxInterruptHandler)(void);
/* cppcheck-suppress misra-c2012-8.9 */
static void (*UART1_TxCompleteInterruptHandler)(void) = NULL;
void (*UART1_RxInterruptHandler)(void);
static void (*UART1_RxCompleteInterruptHandler)(void) = NULL;

static void UART1_DefaultFramingErrorCallback(void);
static void UART1_DefaultOverrunErrorCallback(void);
static void UART1_DefaultParityErrorCallback(void);
void UART1_TransmitISR (void);
void UART1_ReceiveISR(void);

/**
  Section: UART1  APIs
*/

void UART1_Initialize(void)
{
    PIE4bits.U1RXIE = 0;   
    UART1_RxInterruptHandler = UART1_ReceiveISR; 
    PIE4bits.U1TXIE = 0; 
    UART1_TxInterruptHandler = UART1_TransmitISR; 

    // Set the UART1 module to the options selected in the user interface.

    //RXCHK disabled; 
    U1RXCHK = 0x0;
    //TXCHK disabled; 
    U1TXCHK = 0x0;
    //P1L 0x0; 
    U1P1L = 0x0;
    //P1H 0x0; 
    U1P1H = 0x0;
    //P2L 0x0; 
    U1P2L = 0x0;
    //P2H 0x0; 
    U1P2H = 0x0;
    //P3L 0x0; 
    U1P3L = 0x0;
    //P3H 0x0; 
    U1P3H = 0x0;
    //MODE Asynchronous 8-bit mode; RXEN enabled; TXEN enabled; ABDEN disabled; BRGS high speed; 
    U1CON0 = 0xB0;
    //SENDB disabled; BRKOVR disabled; RXBIMD Set RXBKIF on rising RX input; WUE disabled; ON enabled; 
    U1CON1 = 0x80;
    //FLO off; TXPOL not inverted; C0EN disabled; STP Transmit 1Stop bit, receiver verifies first Stop bit; RXPOL not inverted; RUNOVF RX input shifter continues; 
    U1CON2 = 0x80;
    //BRGL 130; 
    U1BRGL = 0x82;
    //BRGH 6; 
    U1BRGH = 0x6;
    //TXBE empty; STPMD in middle of first Stop bit; TXWRE No error; 
    U1FIFO = 0x2E;
    //ABDIE disabled; ABDIF Auto-baud not enabled or not complete; WUIF WUE not enabled by software; 
    U1UIR = 0x0;
    //TXCIF equal; RXFOIF not overflowed; RXBKIF No Break detected; FERIF no error; CERIF No Checksum error; ABDOVF Not overflowed; PERIF no parity error; TXMTIF empty; 
    U1ERRIR = 0x80;
    //TXCIE disabled; RXFOIE disabled; RXBKIE disabled; FERIE disabled; CERIE disabled; ABDOVE disabled; PERIE disabled; TXMTIE disabled; 
    U1ERRIE = 0x0;

    UART1_FramingErrorCallbackRegister(UART1_DefaultFramingErrorCallback);
    UART1_OverrunErrorCallbackRegister(UART1_DefaultOverrunErrorCallback);
    UART1_ParityErrorCallbackRegister(UART1_DefaultParityErrorCallback);

    uart1RxLastError.status = 0;  
    uart1TxHead = 0;
    uart1TxTail = 0;
    uart1TxBufferRemaining = sizeof(uart1TxBuffer);
    uart1RxHead = 0;
    uart1RxTail = 0;
    uart1RxCount = 0;
    PIE4bits.U1RXIE = 1;
}

void UART1_Deinitialize(void)
{
    PIE4bits.U1RXIE = 0;   
    PIE4bits.U1TXIE = 0;
    U1RXB = 0x00;
    U1RXCHK = 0x00;
    U1TXB = 0x00;
    U1TXCHK = 0x00;
    U1P1L = 0x00;
    U1P1H = 0x00;
    U1P2L = 0x00;
    U1P2H = 0x00;
    U1P3L = 0x00;
    U1P3H = 0x00;
    U1CON0 = 0x00;
    U1CON1 = 0x00;
    U1CON2 = 0x00;
    U1BRGL = 0x00;
    U1BRGH = 0x00;
    U1FIFO = 0x00;
    U1UIR = 0x00;
    U1ERRIR = 0x00;
    U1ERRIE = 0x00;
}

void UART1_Enable(void)
{
    U1CON1bits.ON = 1; 
}

void UART1_Disable(void)
{
    U1CON1bits.ON = 0; 
}

void UART1_TransmitEnable(void)
{
    U1CON0bits.TXEN = 1;
}

void UART1_TransmitDisable(void)
{
    U1CON0bits.TXEN = 0;
}

void UART1_ReceiveEnable(void)
{
    U1CON0bits.RXEN = 1;
}

void UART1_ReceiveDisable(void)
{
    U1CON0bits.RXEN = 0;
}

void UART1_SendBreakControlEnable(void)
{
    U1CON1bits.SENDB = 1;
}

void UART1_SendBreakControlDisable(void)
{
    U1CON1bits.SENDB = 0;
}

void UART1_AutoBaudSet(bool enable)
{
    if(enable)
    {
        U1CON0bits.ABDEN = 1; 
    }
    else
    {
      U1CON0bits.ABDEN = 0;  
    }
}


bool UART1_AutoBaudQuery(void)
{
    return (bool)U1UIRbits.ABDIF; 
}

void UART1_AutoBaudDetectCompleteReset(void)
{
    U1UIRbits.ABDIF = 0; 
}

bool UART1_IsAutoBaudDetectOverflow(void)
{
    return (bool)U1ERRIRbits.ABDOVF; 
}

void UART1_AutoBaudDetectOverflowReset(void)
{
    U1ERRIRbits.ABDOVF = 0; 
}

void UART1_TransmitInterruptEnable(void)
{
    PIE4bits.U1TXIE = 1;
}

void UART1_TransmitInterruptDisable(void)
{ 
    PIE4bits.U1TXIE = 0;
}

void UART1_ReceiveInterruptEnable(void)
{
    PIE4bits.U1RXIE = 1;
}
void UART1_ReceiveInterruptDisable(void)
{
    PIE4bits.U1RXIE = 0;
}

bool UART1_IsRxReady(void)
{
    return (uart1RxCount ? true : false);
}

bool UART1_IsTxReady(void)
{
    return (uart1TxBufferRemaining ? true : false);
}

bool UART1_IsTxDone(void)
{
    return U1ERRIRbits.TXMTIF;
}

size_t UART1_ErrorGet(void)
{
    uart1RxLastError.status = uart1RxStatusBuffer[(uart1RxTail) & UART1_RX_BUFFER_MASK].status;

    return uart1RxLastError.status;
}

uint8_t UART1_Read(void)
{
    uint8_t readValue  = 0;
    uint8_t tempRxTail;
    
    readValue = uart1RxBuffer[uart1RxTail];
    tempRxTail = (uart1RxTail + 1U) & UART1_RX_BUFFER_MASK; // Buffer size of RX should be in the 2^n  
    uart1RxTail = tempRxTail;  
    PIE4bits.U1RXIE = 0; 
    if(0U != uart1RxCount)
    {
        uart1RxCount--;
    }
    PIE4bits.U1RXIE = 1;
    return readValue;
}

void __interrupt(irq(IRQ_U1RX), base(8)) UART1_Receive_Vector_ISR(void)
{   
    UART1_ReceiveISR();
}

void UART1_ReceiveISR(void)
{
    uint8_t regValue;
	uint8_t tempRxHead;
    // use this default receive interrupt handler code
    uart1RxStatusBuffer[uart1RxHead].status = 0;

    if(true == U1ERRIRbits.FERIF)
    {
        uart1RxStatusBuffer[uart1RxHead].ferr = 1;
        if(NULL != UART1_FramingErrorHandler)
        {
            UART1_FramingErrorHandler();
        } 
    }
    if(true == U1ERRIRbits.RXFOIF)
    {
        uart1RxStatusBuffer[uart1RxHead].oerr = 1;
        if(NULL != UART1_OverrunErrorHandler)
        {
            UART1_OverrunErrorHandler();
        }   
    }   
    if(true == U1ERRIRbits.PERIF)
    {
        uart1RxStatusBuffer[uart1RxHead].perr = 1;
        if(NULL != UART1_ParityErrorHandler)
        {
            UART1_ParityErrorHandler();
        }   
    }  
 
    regValue = U1RXB;
    
    tempRxHead = (uart1RxHead + 1U) & UART1_RX_BUFFER_MASK;
    if (tempRxHead == uart1RxTail) 
    {
		// ERROR! Receive buffer overflow 
	} 
    else
    {
        uart1RxBuffer[uart1RxHead] = regValue;
		uart1RxHead = tempRxHead;
		uart1RxCount++;
	}   
    
    if(NULL != UART1_RxCompleteInterruptHandler)
    {
        (*UART1_RxCompleteInterruptHandler)();
    } 
}

void UART1_Write(uint8_t txData)
{
    uint8_t tempTxHead;
    
    if(0 == PIE4bits.U1TXIE)
    {
        U1TXB = txData;
    }
    else if(0U < uart1TxBufferRemaining) // check if at least one byte place is available in TX buffer
    {
       uart1TxBuffer[uart1TxHead] = txData;
       tempTxHead = (uart1TxHead + 1U) & UART1_TX_BUFFER_MASK;

       uart1TxHead = tempTxHead;
       PIE4bits.U1TXIE = 0; //Critical value decrement
       uart1TxBufferRemaining--; // one less byte remaining in TX buffer
    }
    else
    {
        //overflow condition; uart1TxBufferRemaining is 0 means TX buffer is full
    }
    PIE4bits.U1TXIE = 1;
}

void __interrupt(irq(IRQ_U1TX), base(8)) UART1_Transmit_Vector_ISR(void)
{   
    UART1_TransmitISR();
}

void UART1_TransmitISR(void)
{
    uint8_t tempTxTail;
    // use this default transmit interrupt handler code
    if(sizeof(uart1TxBuffer) > uart1TxBufferRemaining) // check if all data is transmitted
    {
       U1TXB = uart1TxBuffer[uart1TxTail];
       tempTxTail = (uart1TxTail + 1U) & UART1_TX_BUFFER_MASK;
       
       uart1TxTail = tempTxTail;
       uart1TxBufferRemaining++; // one byte sent, so 1 more byte place is available in TX buffer
    }
    else
    {
        PIE4bits.U1TXIE = 0;
    }
    if(NULL != UART1_TxCompleteInterruptHandler)
    {
        (*UART1_TxCompleteInterruptHandler)();
    }
}





static void UART1_DefaultFramingErrorCallback(void)
{
    
}

static void UART1_DefaultOverrunErrorCallback(void)
{
    
}

static void UART1_DefaultParityErrorCallback(void)
{
    
}

void UART1_FramingErrorCallbackRegister(void (* callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
        UART1_FramingErrorHandler = callbackHandler;
    }
}

void UART1_OverrunErrorCallbackRegister(void (* callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
        UART1_OverrunErrorHandler = callbackHandler;
    }    
}

void UART1_ParityErrorCallbackRegister(void (* callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
        UART1_ParityErrorHandler = callbackHandler;
    } 
}
void UART1_RxCompleteCallbackRegister(void (* callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
       UART1_RxCompleteInterruptHandler = callbackHandler; 
    }   
}

void UART1_TxCompleteCallbackRegister(void (* callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
       UART1_TxCompleteInterruptHandler = callbackHandler;
    }   
}

