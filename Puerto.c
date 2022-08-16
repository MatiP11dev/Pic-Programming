/*
 * File:   Puerto.c
 * Author: Alumno
 *
 * Created on 10 de agosto de 2022, 14:27
 */

#define _XTAL_FREQ 2000000
#include <xc.h>
// CONFIG1H
#pragma config OSC = INTIO7     // Oscillator Selection bits (Internal oscillator block, CLKOUT function on RA6, port function on RA7)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#pragma config IESO = ON        // Internal/External Oscillator Switchover bit (Oscillator Switchover mode enabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled and controlled by software (SBOREN is enabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON      // MCLR Pin Enable bit (RE3 input pin enabled; MCLR disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
int tick_count;
int flag=0;
void configuraPWM(){

    //Configura frecuencia del PWM a 488Hz
    PR2 = 0xFF;
    
    //Configura el Ciclo de Trabajo al 50%
    CCPR1L = 0x04;
    CCP1CONbits.DC1B0 = 0;
    CCP1CONbits.DC1B1 = 0;
    
    //Configura el pin RC2 como salida para el PWM1
    TRISCbits.RC2 = 0;
    
    //Configura la pre-escala de 16
    T2CONbits.T2CKPS = 3;
    
   
    //Configura el funcionamiento como PWM
    CCP1CONbits.CCP1M0 = 1;
    CCP1CONbits.CCP1M1 = 1;
    CCP1CONbits.CCP1M2 = 1;
    CCP1CONbits.CCP1M3 = 1;
    
    //LImpia el registro del TImer2 que guarda la cuenta
    TMR2 = 0;
    
    //Activa el funcionamiento del TImer2
    T2CONbits.TMR2ON = 1;

}
void __interrupt(high_priority) tcInt(void)
{
    if (TMR0IE && TMR0IF) {  // any timer 0 interrupts?
        TMR0IF=0;
        ++tick_count;
        if(flag==0){
            PORTBbits.RB0=0;
            flag=1;
        }
        else{
            PORTBbits.RB0=1;
            flag=0;
        }
    }
    if (TMR1IE && TMR1IF) {  // any timer 1 interrupts?
        TMR1IF=0;
        tick_count += 100;
    }
    // process other interrupt sources here, if required
    return;
}
void main(void) {
    char contador=0;
    TRISB=0;
    TMR0L = 254;               // Timer0 preload value

   T0CON = 0xC6;// = 0b11000111  Set TMR0 to 8bit mode and prescaler to 

   INTCONbits.GIE=1;             // Enable global interrupt//INTCON.B7
   INTCONbits.TMR0IE=1;
   // You can also set GIE_bit like INTCON.B7 = 1; and INTCON.GIE = 1;
   configuraPWM();
  

   // Equivalent to INTCON.B5 = 1; and INTCON.TMR0IE = 1;

    while(1){
//        PORTBbits.RB0=1;
//        __delay_ms(1000);
//        PORTBbits.RB0=0;
//        __delay_ms(1000);
    }
    return;
}