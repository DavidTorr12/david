//CONFIGBITS.h
// PIC18F4550 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1L
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = INTOSC_EC // Oscillator Selection bits (Internal oscillator, CLKO function on RA6, EC used by USB (INTCKO))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = OFF        // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
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
#pragma config STVREN = OFF     // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
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

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

//LIB4550.h
//PORT A
#define PORTA_CONF TRISA
#define PORTA_WRITE LATA
#define PORTA_READ PORTA

//PORT B
#define PORTB_CONF TRISB
#define PORTB_WRITE LATB
#define PORTB_READ PORTB

//PORT C
#define PORTC_CONF TRISC
#define PORTC_WRITE LATC
#define PORTC_READ PORTC

//PORT D
#define PORTD_CONF TRISD
#define PORTD_WRITE LATD
#define PORTD_READ PORTD

#define LED_BUILTIN LATAbits.LATA4
#define buttonA PORTCbits.RC4
#define buttonB PORTCbits.RC5
#define botonReset PORTEbits.RE3

#define BUZZER_CONF TRISCbits.TRISC2
#define BUZZER_WRITE LATCbits.LATC2

#define BUZZER_CONF TRISEbits.TRISE0
#define BUZZER_WRITE LATEbits.LATE0

//Definir Puertos
#define _PB0 0
#define _PB1 1
#define _PB2 2
#define _PB3 3
#define _PB4 4
#define _PB5 5
#define _PB6 6
#define _PB7 7

#define _PD0 8
#define _PD1 9
#define _PD2 10
#define _PD3 11
#define _PD4 12
#define _PD5 13
#define _PD6 14
#define _PD7 15

#define _PC0 16
#define _PC1 17
#define _PC2 18
#define _PC6 19
#define _PC7 20



#define _PE0 21
#define _PE1 22
#define _PE2 23

//Entradas analógicas
#define _AN0 0
#define _AN1 1
#define _AN2 2
#define _AN3 3
#define _AN4 4


#define OUTPUT 0
#define INPUT 1


// Variables
//bool buttonAflag;
unsigned int buttonApressedcounter;
unsigned int buttonApressedcounter2;
//bool buttonBflag;
unsigned int buttonBpressedcounter;
unsigned int buttonBpressedcounter2;
char digitcounter = 0;
char units = 0;
char tens = 0;
char hundreds = 0;
char thousands = 0;

// FUNCIONES
void ONbuttonA_pressed(void);
void ONbuttonB_pressed(void);
void every5ms(void);
void __interrupt() myint(void);
void tmr0init(void);
void ADCinit(void);
void PWMinit(void);
void analogWrite(char pin, int dutyc);
void system_inicializacion(void);
void pinMode(char puerto, char modo);
void buzzer(int frec, long tiempo);
void serialinit(void);
void serialout(char datax);
char serialin(void);
void serialout_st(char *str);
int get_temp(void);
int get_lightlevel(void);
void apagado(int time);
int analogRead(char analogpin);


//LIB4550.c
#include <stdio.h>
#include <stdlib.h>
#include <xc.h>

#define _XTAL_FREQ 4000000

#define speaker LATEbits.LATE0

char buttonAflag = 0;
unsigned int buttonApressedcounter = 0;
unsigned int buttonApressedcounter2 = 0;

char buttonBflag = 0;
unsigned int buttonBpressedcounter = 0;
unsigned int buttonBpressedcounter2 = 0;

void __interrupt() myint(void){
    
    ONbuttonA_pressed();
    ONbuttonB_pressed();
    
    INTCONbits.TMR0IF =0; // Reseteamos la bandera de interrupción
    TMR0L = 236;
    return;
}

void ADCinit(){
    TRISA = TRISA | 0b11101111; //Todo el puerto A como entrada
    ADCON1 = 0b00001010; // Voltajes de referencia VSS y VDD (5 y 0 volts) / RA0 como entrada analógica
    ADCON0 = 0b00000001; // Selección de AN0 como canal de entrada analógica
    ADCON2 = 0b10000100; //Justificación a la derecha / Reloj de conversión FOSC/4
    ADCON0bits.ADON = 1; //Convertidor AD encendido
}

void system_inicializacion (void){
    OSCCON &= 0b10001111;
    OSCCON |= 0b01100000;
    
    tmr0init();
    UCON = 0b11110111; //apaga módulo USB, activar RC4 y RC5
    UCFGbits.UTRDIS = 1;
    
    ADCinit();
    PWMinit();
    
    TRISAbits.TRISA4 = 0; //LET BUILTIN
}

void tmr0init(void){
    INTCONbits.GIE = 1; // habilitar las interrupciones globales (configurar PEIE = 0)
INTCONbits.PEIE = 0; // Deshablitar Interrupciones de periféricos.
INTCONbits.TMR0IE = 1; //Habilitar interrupción por desbordamiento de TMR0
INTCONbits.INT0IE = 0; // Deshabilitar interrupciones externas.
INTCONbits.RBIE = 0; //Deshabilitar interrupción por puerto RB
INTCONbits.TMR0IF =0; // Limpiar la bandera de desbordamiento de TMR0
RCONbits.IPEN = 0; // Deshabilitar prioridades en las interrupciones
T0CONbits.TMR0ON = 1; // Habilitar tmr0
T0CONbits.T08BIT = 1; //Configurar timer a 8 bits (puede ser de 16 bits)
T0CONbits.T0CS = 0; // Seleccionar que el timer0 se incrementa por ciclo de reloj interno
T0CONbits.PSA = 0; // Utilizar un prescaler (para hacer más largo el timpo de cuenta del timer).
//Los siguientes 3 bits controlan el preescaler, en este caso 1:256
T0CONbits.T0PS0 = 1; 
T0CONbits.T0PS1 = 1;
T0CONbits.T0PS2 = 1;
TMR0L = 236;
/*
Para calcular un tiempo específico se utiliza la siguiente fórmula
* 
* T = 4*(1/FOSC)(256-TMR0L)(Preescaler)
* Por ejemplo, si no asignamos ningún valor al registro TMR0L, siempre iniciara desde 
* cero?
*
T = 4(1/4Mhz)(256-0)(256) = 65ms
*
*Ó bien, si asignamos un valor a TMR0L dentro de la rutina de interrupción, se puede 
*conseguir otro valor. Por ejemplo si TMR0L=156, entonces?.
*
T = 4(1/4Mhz)(256-156)(256) = 25ms
*/
}

void pinMode (char puerto, char mode){
    if(puerto == _PB0) TRISBbits.TRISB0 = mode;
    if(puerto == _PB1) TRISBbits.TRISB1 = mode;
    if(puerto == _PB2) TRISBbits.TRISB2 = mode;
    if(puerto == _PB3) TRISBbits.TRISB3 = mode;
    if(puerto == _PB4) TRISBbits.TRISB4 = mode;
    if(puerto == _PB5) TRISBbits.TRISB5 = mode;
    if(puerto == _PB6) TRISBbits.TRISB6 = mode;
    if(puerto == _PB7) TRISBbits.TRISB7 = mode;
       
    if(puerto == _PC0) TRISCbits.TRISC0 = mode;
    if(puerto == _PC1) TRISCbits.TRISC1 = mode;
    if(puerto == _PC2) TRISCbits.TRISC2 = mode;
    if(puerto == _PC6) TRISCbits.TRISC6 = mode;
    if(puerto == _PC7) TRISCbits.TRISC7 = mode;
    
    if(puerto == _PD0) TRISDbits.TRISD0 = mode;
    if(puerto == _PD1) TRISDbits.TRISD1 = mode;
    if(puerto == _PD2) TRISDbits.TRISD2 = mode;
    if(puerto == _PD3) TRISDbits.TRISD3 = mode;
    if(puerto == _PD4) TRISDbits.TRISD4 = mode;
    if(puerto == _PD5) TRISDbits.TRISD5 = mode;
    if(puerto == _PD6) TRISDbits.TRISD6 = mode;
    if(puerto == _PD7) TRISDbits.TRISD7 = mode;
    
    if(puerto == _PE0) TRISEbits.TRISE0 = mode;
    if(puerto == _PE1) TRISEbits.TRISE1 = mode;
    if(puerto == _PE2) TRISEbits.TRISE2 = mode;    
}

void buzzer(int frec, long tiempo){ 
    TRISEbits.TRISE0 = 0;
    int delayus = 0;
    int bloque = 0;
    long repeticiones = 0;
    delayus = 1000000/(frec*2); 
    bloque = delayus/18;
    repeticiones = (tiempo*1000)/(delayus*2);   
    for(int a=0; a<repeticiones; a++){ 
        speaker = 1;
        for(int i=0; i<bloque; i++){}
        speaker = 0;
        for(int i=0; i<bloque; i++){}
    } 
}

//inicia comunicación serial asíncrona de 8 bits 1200 bauds
void serialinit(){
    TRISCbits.TRISC6 = 1;
    TRISCbits.TRISC7 = 1;

    //TXSTAbits.CSRC = 0; //Clock source select bit
    TXSTAbits.TX9 = 0; //8 bit transmission
    TXSTAbits.TXEN = 0; //Transmit enabled
    TXSTAbits.SYNC = 0; //Asynchronous mode
    TXSTAbits.SENDB = 1; //Send sync Break on next...
    //...transmission (cleared by...
    //...hardware upon completion).
    TXSTAbits.BRGH = 0; //Low speed rate


    RCSTAbits.SPEN = 1;
    RCSTAbits.RX9 = 0;
    RCSTAbits.CREN = 1; //enables receiver
    RCSTAbits.ADDEN = 0; //Disables address detection
    RCSTAbits.FERR = 0; //No overrun error


    BAUDCONbits.ABDOVF = 0;
    BAUDCONbits.RXDTP = 0;
    BAUDCONbits.TXCKP = 0;
    BAUDCONbits.BRG16 = 0;
    BAUDCONbits.ABDEN = 0;

    SPBRG = 51;
    TXREG = 0b01110111;
}

void serialout(char datax){
    TXSTAbits.TXEN = 1;
    while(TXSTAbits.TRMT == 0);
    TXREG = datax;
}

char serialin(){
    if (PIR1bits.RCIF) {
        return RCREG;
        PIR1bits.RCIF = 0;
    }
    else return('\0');
}

void serialout_st(char *str){
    while(*str !='\0'){
        serialout(*str);
        str++;
    }
}



int get_temp(){
    ADCON0bits.CHS = _AN0; //seleccionar canal 0 
    unsigned int aux = 0;   
    ADCON0bits.GODONE = 1; //Iniciar la conversión Analógico-Digital
    while (ADCON0bits.GODONE == 1){} //Esperar que termine la conversión.

    aux = ADRESH << 8; 
    aux |= ADRESL; 
    //__delay_ms(100);
    aux = aux*4.9; //convertir a grados * 10
    return aux; 
}
int get_lightlevel(){
    ADCON0bits.CHS = _AN1; //seleccionar canal 0 
    unsigned int aux = 0;   
    ADCON0bits.GODONE = 1; //Iniciar la conversión Analógico-Digital
    while (ADCON0bits.GODONE == 1){} //Esperar que termine la conversión.

    aux = ADRESH << 8; 
    aux |= ADRESL; 
    return aux; 
}
int analogRead(char analogpin){
    ADCON0bits.CHS = analogpin; //seleccionar canal 0 
    unsigned int aux = 0;   
    ADCON0bits.GODONE = 1; //Iniciar la conversión Analógico-Digital
    while (ADCON0bits.GODONE == 1){} //Esperar que termine la conversión.

    aux = ADRESH << 8; 
    aux |= ADRESL; 
    aux= aux*4.8;
    return aux; 
}

void PWMinit(){
    CCP1CON = 0b00001100;
    CCP2CON = 0b00001100;
    //ESTABLECER LA FRECUENCIA DE OPERACIÓN (UTILIZANDO LA FÓRMULA)
    PR2 = 124; //Para 500Hz con reloj de 4MHz
    //ESTABLECER EL PREESCALER, EN ESTE CASO 1:16
    T2CONbits.T2CKPS0 = 0;
    T2CONbits.T2CKPS1 = 1;
    //ACTIVAR EL TIMER2
    T2CONbits.TMR2ON = 1;
    //ESTABLECER EL CICLO DE TRABAJO,10 BITS EN TOTAL, EN ESTE CASO EL
    //NÚMERO 512 EN BINARIO: 10000000000
    CCPR1L = 0b00000000;
    CCP1CONbits.DC1B1 = 0;
    CCP1CONbits.DC1B0 = 0;

    CCPR2L = 0b00000000;
    CCP2CONbits.DC2B1 = 0;
    CCP2CONbits.DC2B0 = 0;
    //CONFIGURAR PUERTO C COMO SALIDA (SÓLO SE UTILIZARÁ RC2)
    TRISCbits.TRISC1 = 0;
    TRISCbits.TRISC2 = 0;
}

void analogWrite(char pin, int dutyc){ 
    int DATO = 0;
    DATO = 1.96*dutyc;
    
    if (pin == _PC1){
        CCPR2L = DATO >> 2;
        CCP2CONbits.DC2B = DATO & 0b11;
    }
    
    if (pin == _PC2){
        CCPR1L = DATO >> 2;
        CCP1CONbits.DC1B = DATO & 0b11;
    }
}

//LEDS DEL PUERTO RB BOTON B
#define LED1 LATBbits.LATB0
#define LED2 LATBbits.LATB1
#define LED3 LATBbits.LATB2
#define LED4 LATBbits.LATB3
#define LED5 LATBbits.LATB4
#define LED6 LATBbits.LATB5
#define LED7 LATBbits.LATB6
#define LED8 LATBbits.LATB7

//contador de pulsaciones boton A
int contador1 = 0;
//contador de pulsaciones boton B
int contador2 = 0;
//Contador de milisegundos transcurridos
int contador3 = 1;
//Tiempo para que se detenga la cuenta regresiva
//int stop = 0;
//Bandera para corroborar la activacion de la cuenta regresiva
int bandera = 0;

//MAIN
void main() {
    system_inicializacion();
    TRISEbits.TRISE0 = 0; //E0 como salida
    TRISCbits.TRISC1 = 0; //RC1 como salida
    
    serialinit();
    PORTB_CONF = 0;
    PORTD_CONF = 0;
    //Inicializar en cero el circuito
    analogWrite(_PC1,0);
    
    LED_BUILTIN = 1;

        LED1 = 0;
        LED2 = 0;
        LED3 = 0;
        LED4 = 0;
        LED5 = 0;
        LED6 = 0;
        LED7 = 0;
        LED8 = 0;   
         
    while (1) {
    __delay_ms(1);
   
    ONbuttonA_pressed(); 
    ONbuttonB_pressed();
//Checar cuando se haya iniciado la cuenta regresiva
    if(bandera == 1){
         contador3--;
         apagado(contador3);
    }
    
     if(contador3 == 0){
         contador1=0;
         contador2=0;
         LED1 = 0;
         LED2 = 0;
         LED3 = 0;
         LED4 = 0;
         LED5 = 0;
         LED6 = 0;
         LED7 = 0;
         LED8 = 0;
         analogWrite(_PC1,0);
         //Triple bip de terminacion
         buzzer(2000,100);
         __delay_ms(10);
         buzzer(1800,100);
         __delay_ms(10);
         buzzer(1600,100);
         
         bandera = 0;
         contador3 = 1;
     }
    }
   
}
 //Funcion de orden en que los leds se apagan
void apagado(int time){
    if(time == 5000){
        LED6 = 0;
    }
    
     if(time == 10000){
        LED7 = 0;
    }
    
     if(time == 15000){
        LED8 = 0;
    }
}

void ONbuttonA_pressed() {
    
    if (buttonA && buttonAflag) {
        buttonApressedcounter++;
        buttonApressedcounter2++;
    }


    if (buttonA && !buttonAflag) { //as soon as it's pressed
        buttonAflag = 1;
        buttonApressedcounter = 0;
        buttonApressedcounter2 = 0;
        //your code here
        
        //Control de velocidades del ventilador
        contador1++;
        switch(contador1){
           case 1: analogWrite(_PC1,100);buzzer(2000,100);LED1=1;break;
           case 2: analogWrite(_PC1,180);buzzer(2000,100);LED1=0;LED2=1; break;
           case 3: analogWrite(_PC1,250);buzzer(2000,100);LED2=0;LED3=1; break;
           case 4: analogWrite(_PC1,0);buzzer(2000,100);contador1=0;LED3=0;break;
           
    }
    }

    if (!buttonA && buttonAflag && buttonApressedcounter2 < 399) { //relase button before 2 sec
        buttonAflag = 0;

        //your code here
       
        

    }

    if (buttonA && buttonApressedcounter == 399) { //keep pressing during 2 sec
        buttonApressedcounter = 0;
        buttonApressedcounter2 = 400;
        //your code here
     

    }
    if (!buttonA) buttonAflag = 0;

}

void ONbuttonB_pressed() {

    if (buttonB && buttonBflag) {
        buttonBpressedcounter++;
        buttonBpressedcounter2++;
    }


    if (buttonB && !buttonBflag) { //as soon as it's pressed
        buttonBflag = 1;
        buttonBpressedcounter = 0;
        buttonBpressedcounter2 = 0;
        //your code here
        
        
        contador2++;
        bandera = 1;
        switch(contador2){
           case 1: contador3 = 5000;buzzer(1800,100);  LED5=1;break;
           case 2: contador3 = 10000;buzzer(1800,100); LED5=1;LED6=1;break;
           case 3: contador3 = 15000;buzzer(1800,100); LED5=1;LED6=1;LED7=1;break;
           case 4: contador3 = 20000;buzzer(1800,100); LED5=1;LED6=1;LED7=1;LED8=1;break;
           case 5: contador3 = 1; bandera = 0; contador2 = 0;LED5=0;LED6=0;LED7=0;LED8=0;break;
           
    }
       
        }


    if (!buttonB && buttonBflag && buttonBpressedcounter2 < 399) { //relase button before 2 sec
        buttonBflag = 0;

        //your code here


    }

    if (buttonB && buttonBpressedcounter == 399) { //keep pressing during 2 sec
        buttonBpressedcounter = 0;
        buttonBpressedcounter2 = 400;
        //your code here

    }
    if (!buttonB) buttonBflag = 0;

}

void every5ms() {

}22
