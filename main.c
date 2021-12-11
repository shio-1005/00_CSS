#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <spi.h>
#include <MCP2515.h>
#include <fram.h>
#include "usart.h"

// CONFIG1
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = ON       // RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown Out Reset Selection bits (BOR enabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR21V   // Brown-out Reset Selection bit (Brown-out Reset set to 2.1V)
#pragma config WRT   = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#define _XTAL_FREQ      20000000
#define _CAN_BAUDRATE   2

#define _RXB0D0         0b01100110

/*--Prtotype--*/
void CONFIG();

/*--Grobal Variables--*/
char flag_id[4] = {0b00000000, 0b00001000, 0b00000000, 0b00000000};
//char ack = 1;
//char id[4] = {0b00000000, 0b00001000, 0b00000000, 0b00000000}; 
volatile char* r_data;
char rx_int;
volatile char* w_data; 
volatile char flag;
volatile char* rx_sidh, rx_sidl, rx_eid8, rx_eid0;
//char tx = 1;

//volatile int i;
volatile char w_cnt = 0;
volatile char *r_cnt = 0;

void main(void)
{ 
    CONFIG();
    _usart_init();
    __delay_ms(100);
    
    
    while(1)
    {  
        rx_int = Read(_CANINTF);                                                                //受信バッファ確認
        if((rx_int & _Flagbit0) == 0b00000001)                                                  //受信待ち
        {
            //RC6 = 1;

            Write(_CANINTF, 0b00000000);                                                        //受信フラグクリア
            
            rx_sidh = Read(_RXB0SIDH);
            rx_sidl = Read(_RXB0SIDL);
            rx_eid8 = Read(_RXB0EID8);
            rx_eid0 = Read(_RXB0EID0);
            
            if((rx_sidl & _Flagbit0) == 0b00000001)                                             //Writeなら
            {  
                w_data = Read_RX_Buffer(_F_RXB0D0, 8);
                Wren();                                                                         //書き込み許可
                WRITE(rx_eid8, rx_eid0, &w_data[0], 8);
                
                /* ターミナルにID出力 
                
                if(rx_eid8 == 1)
                {
                    while(!TXSTAbits.TRMT);
                    TXREG = '1';
                    //while(!TXSTAbits.TRMT);
                    /*
                    TXREG = 13;                                                                 //CR(行頭復帰)
                    while(!TXSTAbits.TRMT);
                    TXREG = 10;                                                                 //LF(改行)
                    while(!TXSTAbits.TRMT);
                    
                }else if(rx_eid8 == 2)
                {
                    while(!TXSTAbits.TRMT);
                    TXREG = '2';
                    //while(!TXSTAbits.TRMT);                                                   
                }else if(rx_eid8 == 3)
                {
                    while(!TXSTAbits.TRMT);
                    TXREG = '3';
                    //while(!TXSTAbits.TRMT);
                }else if(rx_eid8 == 4)
                {
                    while(!TXSTAbits.TRMT);
                    TXREG = '4';
                    //while(!TXSTAbits.TRMT);
                }else if(rx_eid8 == 5)
                {
                    while(!TXSTAbits.TRMT);
                    TXREG = '5';
                    //while(!TXSTAbits.TRMT);
                }else if(rx_eid8 == 6)
                {
                    while(!TXSTAbits.TRMT);
                    TXREG = '6';
                    //while(!TXSTAbits.TRMT);
                }
                */
                /* ACKの送信 */
                Write(_TXB0DLC , 0b00000001);                                                   //データサイズ1byte
                Load_TX_ID(_F_TXB0SIDH, rx_sidh, rx_sidl, rx_eid8, rx_eid0+1);                    //ID
                Load_TX_Data(_F_TXB0D0, 1, &w_cnt);                                            
                RTS0_CSS(2);                                                                    //送信要求
                
                if(w_cnt == 255)
                {
                    w_cnt = 0;
                }else{
                    w_cnt++;
                }
                
                RC6 = 0;

            }else if((rx_sidl & _Flagbit0) == 0b00000000)                                                                          //Readなら
            {
                r_data = READ(rx_eid8, rx_eid0, 8);                                             //メモリからモード読み出し
                r_data[7] = r_cnt;
                //r_data[5] = TEC;
                Write(_TXB0DLC , 0b00001000);                                                   //データサイズ8byte
                Load_TX_ID(_F_TXB0SIDH, 0b00000000, 0b00001000, 0b00000000, rx_sidh);                                  //ID:
                Load_TX_Data(_F_TXB0D0, 8, r_data);                                         
                RTS0_CSS(2);                                                                    //送信要求
                
                if(r_cnt[0] == 255)
                {
                    r_cnt = 0;
                }else{
                    r_cnt++;
                }
               
                RC6 = 0;
            }
        }
    }
}

void CONFIG()
{
    OSCCON = 0b01101000;
    ANSEL  = 0b00000000;
    ANSELH = 0b00000000;
    TRISC  = 0b10000000;                                                                        //RC7/RXを入力(1)に設定
    PORTC  = 0b00000000;
    
    spi_init();
    __delay_ms(100);
    
    MCP2515_init(_CAN_BAUDRATE);                                                
    Write(_TXB0DLC , 0b00001000);                                                               //データサイズ8byte
    MCP2515_Open(0);                                                            
}