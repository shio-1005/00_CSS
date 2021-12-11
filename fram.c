/*************************************************************************
 * File         :fram.c                       				 
 * Date         :2020/11/6					 
 * Summary      :Use for fram			
 * CPU type     :PIC16F886						 
 * Compiler     :XC8							 
 * Author       :Akaru Oishi						 
 * Description  :		 
 *              :				 
 *              :		 
 *              :			 
 * ***********************************************************************

/*---File Include---*/
#include <xc.h>
#include <stdio.h>
#include <string.h>
#include <spi.h>
#include <fram.h>

char strage[_FRAM_STRAGE_SIZE];
char id[4];


void Wren(){
    RA4 = 0;
    spi_putc(0b00000110);                                                       //書き込み可能フラグ = 1
    RA4 = 1;
    __delay_us(1);
}

void WRDI(){
    RA4 = 0;
    spi_putc(0b00000100);                                                       //書き込み可能フラグ = 0
    RA4 = 1;
}

char RDSR(){
    char data;
    RA4 = 0;
    spi_putc(0b00000101);                                                       //ステータスレジスタ読み出し
    data = spi_putc(0b10101010);                                                //てきとーな値送って，データを格納
    RA4 = 1;
    __delay_us(1);
    return data;                                                                //ステータスレジスタを返す
}

char *READ(char address_h , char address_l , char bytes)
{
    RA4 = 0;
    spi_putc(0b00000011);                                                       //データ読み出し
    spi_putc(address_h);
    spi_putc(address_l);                                                        //16bitアドレス(最初の3bitは無効)
    for(char i = 0 ; i < bytes ; i++)
    {
        strage[i] = spi_putc(0);
    }
    RA4 = 1;
    __delay_us(1);
    
    return strage;
}

void WRITE(char address_h , char address_l , char *p , char bytes)
{
    RA4 = 0;
    spi_putc(0b00000010);                                                       //データ書き込み
    spi_putc(address_h);
    spi_putc(address_l);
    for(char i = 0 ; i < bytes ; i++)
    {
        spi_putc(*p);
        p++;
    }
    RA4 = 1;
    __delay_us(1);
}

char *RDID()
{
    RA4 = 0;                                                                    //デバイスID読み出し
    spi_putc(0b10011111);
    for(char i=0; i<4; i++)
    {
        id[i] = spi_putc(0);
    }
    RA4 = 1;
    
    return id;
}

