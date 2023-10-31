/* PIC Voltimeter

    0 - 5V Voltimeter using PIC18f4550 A/D converter.

    Author: Guilherme Arruda

    GitHub: https://github.com/ohananoshi/pic_dc_voltimeter

    Created in: 31 oct 2023

    Last updated: 31 oct 2023
*/

//================================= PIC CONFIGURATION BITS =========================================================

// CONFIG1H
#pragma config FOSC = INTOSC_HS // Oscillator Selection bits (XT oscillator (XT))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = ON         // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = ON      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config ICPRT = OFF      // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)


//================================================ HEADERS ====================================================================================

#include <xc.h>
#include <stdint.h>

//lcd configs

#define LCD_4BIT_INTERFACE
#define LCD_D7 PORTDbits.RD7
#define LCD_D6 PORTDbits.RD6
#define LCD_D5 PORTDbits.RD5
#define LCD_D4 PORTDbits.RD4
#define LCD_EN PORTDbits.RD1
#define LCD_RS PORTDbits.RD0

#include "lcd_api.h"
//================================================= CONSTANTS ===================================================================================

//Analogic-Digital converter channels
enum {
    CH_0 = 0x00,
    CH_1 = 0x04,
    CH_2 = 0x08,
    CH_3 = 0x0C,
    CH_4 = 0x10,
    CH_5 = 0x14,
    CH_6 = 0x18,
    CH_7 = 0x1C,
    CH_8 = 0x20,
    CH_9 = 0x24,
    CH_10 = 0x28,
    CH_11 = 0x2C,
    CH_12 = 0x30
}AD_CHANNELS;

//=============================================== FUNCTIONS ==================================================================================

//Return 0-1024 integer value
uint16_t AD_convert(uint8_t AD_channel){

    ADCON0 = (AD_channel | 0x01);
    ADCON0bits.GO_DONE = 1;         // AD conversion start

    while(!ADCON0bits.GO_DONE);     // Wait conversion end
    return (uint16_t)ADRES;         // Returns conversion result
}

float voltage(uint8_t AD_channel){

    float v = (float)AD_convert(AD_channel);

    return ((v*5)/1023);                   // Map voltage value
}

//================================================= MAIN ===========================================================


void main(void){

 OSCCON = 0b01100000;
 PORTD = 0;
 TRISD = 0x00;
 lcd_init(SET_2LINE_MODE);
 
 // PORTA (RA0/AN0 input mode)
 PORTA = 0;
 TRISA = 0x01;
 
 //A/D converter init
 ADCON0 = 0x01;
 ADCON1 = 0x0D;
 ADCON2 = 0x89;

 while (1) {
    lcd_cursor_move(0,0);
    lcd_print("Voltage: %.2f V", voltage(CH_0));
    __delay_ms(100);
 }
}