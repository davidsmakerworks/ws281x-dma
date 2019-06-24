/*
 * WS281x DMA
 * Copyright (c) 2019 David Rice
 * 
 * A minimal example of using DMA on the PIC18F25K42 to drive WS281x LEDs.
 * 
 * This is a work in progress.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// PIC18F25K42 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1L
#pragma config FEXTOSC = OFF    // External Oscillator Selection (Oscillator not enabled)
#pragma config RSTOSC = HFINTOSC_1MHZ// Reset Oscillator Selection (HFINTOSC with HFFRQ = 4 MHz and CDIV = 4:1)

// CONFIG1H
#pragma config CLKOUTEN = OFF   // Clock out Enable bit (CLKOUT function is disabled)
#pragma config PR1WAY = ON      // PRLOCKED One-Way Set Enable bit (PRLOCK bit can be cleared and set only once)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)

// CONFIG2L
#pragma config MCLRE = EXTMCLR  // MCLR Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR )
#pragma config PWRTS = PWRT_OFF // Power-up timer selection bits (PWRT is disabled)
#pragma config MVECEN = ON      // Multi-vector enable bit (Multi-vector enabled, Vector table used for interrupts)
#pragma config IVT1WAY = ON     // IVTLOCK bit One-way set enable bit (IVTLOCK bit can be cleared and set only once)
#pragma config LPBOREN = OFF    // Low Power BOR Enable bit (ULPBOR disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled , SBOREN bit is ignored)

// CONFIG2H
#pragma config BORV = VBOR_2P45 // Brown-out Reset Voltage Selection bits (Brown-out Reset Voltage (VBOR) set to 2.45V)
#pragma config ZCD = OFF        // ZCD Disable bit (ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config DEBUG = OFF      // Debugger Enable bit (Background debugger disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// CONFIG3L
#pragma config WDTCPS = WDTCPS_31// WDT Period selection bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled; SWDTEN is ignored)

// CONFIG3H
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4L
#pragma config BBSIZE = BBSIZE_512// Boot Block Size selection bits (Boot Block size is 512 words)
#pragma config BBEN = OFF       // Boot Block enable bit (Boot block disabled)
#pragma config SAFEN = OFF      // Storage Area Flash enable bit (SAF disabled)
#pragma config WRTAPP = OFF     // Application Block write protection bit (Application Block not write protected)

// CONFIG4H
#pragma config WRTB = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-30000Bh) not write-protected)
#pragma config WRTC = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)
#pragma config WRTSAF = OFF     // SAF Write protection bit (SAF not Write Protected)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored)

// CONFIG5L
#pragma config CP = OFF         // PFM and Data EEPROM Code Protection bit (PFM and Data EEPROM code protection disabled)

// CONFIG5H

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

/**** FREQUENTLY CHANGED CONFIGURATION VALUES START HERE ****/

/* Type of LEDs connected to driver - supported options are WS2811 and WS2812B */
#define WS2812B

/* Maximum LED index connected to this driver (i.e., number of LEDs - 1) */
#define MAX_LED_INDEX 600

/* Initial modes */
#define INIT_MODE MODE_RND_ONE
#define INIT_LEVEL LEVEL_MID

/* Overall maximum values for various brightness levels
 * 
 * For strings or bright strips, usual values are 0xFF, 0x7F, 0x3F 
 * For small rings, usual values are 0x7F, 0x1F, 0x0F 
 */
#define MAX_BRIGHT 0x7F
#define MID_BRIGHT 0x1F
#define DIM_BRIGHT 0x0F

/* Maximums for secondary colors used in fire and ice effects 
 *
 * For strings or bright strips, usual values are 0x7F, 0x3F, 0x1F 
 * For small rings, usual values are 0x3F, 0x0F, 0x08 
 */
#define MAX_BRIGHT_SEC 0x3F
#define MID_BRIGHT_SEC 0x0F
#define DIM_BRIGHT_SEC 0x08

/**** FREQUENTLY CHANGED CONFIGURATION VALUES END HERE ****/

#define BAUD_RATE       206 /* 9600 baud at Fosc = 32 MHz */

#define _XTAL_FREQ      32000000

/* 
 * Macros for putting color values at the appropriate index
 * Note that the WS2812B uses GRB format instead of RGB
 */
#ifdef WS2811
#define RED(x)   (x * 3)
#define GREEN(x) (x * 3) + 1
#define BLUE(x)  (x * 3) + 2
#else
#ifdef WS2812B
#define RED(x)   (x * 3) + 1
#define GREEN(x) (x * 3)
#define BLUE(x)  (x * 3) + 2
#else
#error LED controller type must be specified
#endif
#endif

/* Define available display modes */
typedef enum {
    MODE_STATIC = 0,
    MODE_INC,
    MODE_DEC,
    MODE_RND_ALL,
    MODE_RND_ONE,
    MODE_FIRE,
    MODE_ICE,
    MODE_SPARKS,
    MODE_XMAS
} DISP_MODE;

/* Define brightness levels */
typedef enum {
    LEVEL_DIM = 0,
    LEVEL_MID,
    LEVEL_BRIGHT
} DISP_LEVEL;

volatile bool serial_pending;
volatile uint8_t serial_data;

uint8_t color_data[(MAX_LED_INDEX + 1) * 3];

void __interrupt(irq(U1RX)) uart1_receive_isr(void) {
    serial_data = U1RXB;
    serial_pending = true; /* Set flag so data can be processed in main loop */
    
    PIR3bits.U1RXIF = 0; /* lear UART1 RX interrupt flag */
}

void __interrupt(irq(default)) default_isr(void) {
    asm("RESET"); /* Reset processor on unhandled interrupt */
}

void init_ports(void) {
    /* Set all ports to digital */
    ANSELA = 0x00;
    ANSELB = 0x00;
    ANSELC = 0x00;
    
    /* Set all ports to output */
    TRISA = 0x00;
    TRISB = 0x00;
    TRISC = 0x00;
    
    /* Pull all outputs low */
    LATA = 0x00;
    LATB = 0x00;
    LATC = 0x00;
    
    /* Set TTL input level on RC7 (UART1 RX) because Bluetooth module uses 3.3V*/
    INLVLCbits.INLVLC7 = 0;
}

void init_osc(void) {
    OSCCON1bits.NDIV = 0b0000; /* Oscillator divide radio 1:1 */
    OSCFRQbits.FRQ = 0b0110; /* Set HFINTOSC to 32 MHz */
}

void init_timer2(void) {
    T2CLKbits.CS = 0b0001; /* Set Timer2 clock source to Fosc/4 */
    T2PR = 4; /* 0.625 uSec at Fosc = 32 MHz */
    T2CONbits.ON = 1; /* Enable Timer2 */
}

void init_pps(void) {
    bool status;
    
    status = INTCON0bits.GIE;
    INTCON0bits.GIE = 0;
    
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0;
    
    U1RXPPS = 0b10111; /* UART1 RX on RC7 */
    RA0PPS = 0b000001; /* CLC1OUT on RA0 */
    
    RB0PPS = 0b011111; /* SPI1 SDO on RB0 */
    RB1PPS = 0b011110; /* SPI1 SCK on RB1 */
    
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 1;
    
    INTCON0bits.GIE = status;
}

void init_spi(void) {
    SPI1CLKbits.CLKSEL = 0b0101; /* SPI1 clock source is TMR2_postscaled */
    SPI1BAUD = 0; /* Effective SPI1 clock rate is TMR2_postscaled / 2 */
    
    SPI1CON0bits.MST = 1; /* SPI1 is in master mode */
    SPI1CON0bits.BMODE = 1; /* Send data as soon as it is placed in the TX FIFO */
    SPI1CON1bits.FST = 1; /* Disable SS/SCK sync since SS is not used */
    SPI1CON2bits.TXR = 1; /* Enable SPI1 transmitter */
    SPI1CON2bits.RXR = 0; /* Disable SPI1 receiver - data is transmit-only */
    SPI1CON0bits.EN = 1; /* Enable SPI1 */
}

void init_dma(void) {
    DMA1CON1bits.SMR = 0b00; /* DMA1 source memory region is GPR/SFR */
    DMA1CON1bits.SMODE = 0b01; /* DMA1 source pointer increments after each transfer */
    DMA1CON1bits.SSTP = 1; /* Clear DMA1 SIRQEN when source counter reloads */
    
    DMA1CON1bits.DMODE = 0b00; /* DMA1 destination pointer remains unchanged after transfer */
    DMA1CON1bits.DSTP = 0; /* Do not clear SIRQEN when destination counter reloads */
    
    DMA1SSA = (uint24_t)color_data; /* DMA1 source is color_data array */
    DMA1DSA = (uint24_t)&SPI1TXB; /* DMA1 destination is SPI1 transmit buffer */
    
    DMA1SSZ = (MAX_LED_INDEX + 1) * 3; /* DMA1 source size is length of color_data array */
    DMA1DSZ = 1; /* DMA1 destination size is 1 */
    
    DMA1SIRQ = 21; /* DMA1 source trigger IRQ is SPI1TX */
    
    DMA1CON0bits.EN = 1; /* Enable DMA1 */
}

void init_arbiter(void) {
    bool status;
    
    status = INTCON0bits.GIE;
    INTCON0bits.GIE = 0;
    
    PRLOCK = 0x55;
    PRLOCK = 0xAA;
    
    PRLOCKbits.PRLOCKED = 1; /* Lock system arbiter priorities */
    
    INTCON0bits.GIE = status;
}

void init_uart(void) {
    U1BRG = (uint16_t)BAUD_RATE;
    
    U1CON0bits.RXEN = 1;
    U2CON1bits.ON = 1;
}

void init_clc(void) {
    CLC1SEL0bits.D1S = 0b101011; /* CLC1 input 1 is SDO1 */
    CLC1SEL1bits.D2S = 0b101100; /* CLC1 input 2 is SCK1 */
    CLC1SEL2bits.D3S = 0b011000; /* CLC1 input 3 is PWM5OUT */
    
    CLC1GLS0 = 0x00;
    CLC1GLS1 = 0x00;
    CLC1GLS2 = 0x00;
    CLC1GLS3 = 0x00; /* Gate behavior is undefined at power-on so must be set to zero */
    
    CLC1GLS0bits.G1D1T = 1; /* SDO input to AND gate 1 */
    
    CLC1GLS1bits.G2D2T = 1; /* SCK input to AND gate 1 */
    
    /* nSDO && SCK = n(SDO || nSCK) */
    CLC1GLS2bits.G3D1T = 1;
    CLC1GLS2bits.G3D2N = 1; /* SDO || nSCK input to AND gate 2 */
    
    CLC1GLS3bits.G4D3T = 1; /* PWM5OUT input to AND gate 2 */
    
    CLC1POL = 0x00; /* Clear all inversion bits */
    CLC1POLbits.G3POL = 1; /* Gate 3 n(SDO || nSCK) is inverted to obtain (nSDO && SDK) */
    
    CLC1CONbits.EN = 1; /* Enable CLC1 */
}

/* Initialize PWM generator for WS281x zero-bit pulses */
void init_pwm(void) {
    PWM5DCH = 1;
    PWM5DCL = 0;
    
    PWM5CONbits.EN = 1; /* Enable PWM generator */
}

void init_interrupts() {
    PIE3bits.U1RXIE = 1; /* Enable UART1 receive interrupt */
}

void init_system(void) {
    init_ports();
    init_osc();
    init_pps();
    init_timer2();
    init_spi();
    init_uart();
    init_pwm();
    init_clc();
    init_dma();
    init_arbiter();
    init_interrupts(); 
}

void update_display(void) {
    DMA1CON0bits.SIRQEN = 1; /* Enable DMA1 source IRQ transfer */
}

void main(void) {
    uint16_t i;
    uint32_t j;
    
    volatile uint32_t x;
    volatile uint32_t y;
    volatile uint32_t z;
    
    init_system(); /* Initialize all peripherals and interrupts */
    
    INTCON0bits.GIE = 1; /* Enable global interrupts */
    
    __delay_ms(100);
    
    while (1) {
       for (i = 0; i <= MAX_LED_INDEX; i++) {
           color_data[RED(i)] = rand() % 64;
           color_data[GREEN(i)] = rand() % 64;
           color_data[BLUE(i)] = rand() % 64;
       }
       
       update_display();
       // SPI1TXB = 0xFF;
       
       for (j = 0; j < 1000; j++)
       {
           x = rand();
           y = rand();
           
           z = x / y;
       }
    }
}
