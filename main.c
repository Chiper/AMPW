/**************************************************************
 **************************************************************
 *      POWER MANAGEMENT UNIT FOR POWERING PREAMPLIFIERS      *
 *                  ACOUSTIC EMISSION SENSORS                 *
 **************************************************************
 *                                                            *
 *    		 Author	: Sergey Kostyuchenko                 *
 *    		 E-mail	: Kostuch_S@mail.ru                   *
 *     		 Tel.  	: +7 926 318 5998                     *
 *                                                            *
 **************************************************************
 **************************************************************
 *               PROTOCOL APPEALS TO THE DEVICE               *
 **************************************************************
 *            POWER MANAGEMENT END PULSE GENERATION           *
 *      Right 4 bits (lower bits) responsible for the         *
 *      command to the executive device.                      *
 **************************************************************
 * 0 (0bxxxx0000) - Request information;                      *
 * 1 (0bxxxx0001) - Enable Power MOSFET, disable ground MOSFET*
 * 2 (0bxxxx0010) - Disable Power MOSFET, enable ground MOSFET*
 **************************************************************
 * 3 (0bxxxx0011) - Send pulse   5ms;                         *
 * 4 (0bxxxx0100) - Send pulse  10ms;                         *
 * 5 (0bxxxx0101) - Send pulse  15ms;                         *
 * 6 (0bxxxx0110) - Send pulse  30ms;                         *
 * 7 (0bxxxx0111) - Send pulse  50ms;                         *
 * 8 (0bxxxx1000) - Send pulse  80ms;                         *
 **************************************************************
 *0F (0b00001111) - Command to obtain an address via ADC      *
 **************************************************************
 * 	    ADDRESSING DEVICES AND THEIR CHANNELS             *
 **************************************************************
 *      Left 4 bits (higer bits) are responsible for          *
 *      the address of the device is addressed team.          *
 **************************************************************
 * 0  (0b0000xxxx) -  01 channel (device) address             *
 * 1  (0b0001xxxx) -  02 channel (device) address             *
 * 2  (0b0010xxxx) -  03 channel (device) address             *
 * 3  (0b0011xxxx) -  04 channel (device) address             *
 * 4  (0b0100xxxx) -  05 channel (device) address             *
 * 5  (0b0101xxxx) -  06 channel (device) address             *
 * 6  (0b0110xxxx) -  07 channel (device) address             *
 * 7  (0b0111xxxx) -  08 channel (device) address             *
 * 8  (0b1000xxxx) -  09 channel (device) address             *
 * 9  (0b1001xxxx) -  10 channel (device) address             *
 * A  (0b1010xxxx) -  11 channel (device) address             *
 * B  (0b1011xxxx) -  12 channel (device) address             *
 * C  (0b1100xxxx) -  13 channel (device) address             *
 * D  (0b1101xxxx) -  14 channel (device) address             *
 * E  (0b1110xxxx) -  15 channel (device) address             *
 * F  (0b1111xxxx) -  16 channel (device) address             *
 *************************************************************/

#include <htc.h>
#include <pic.h>
#include <pic16f1825.h>
#include <stdio.h>
#include <stdlib.h>

// UART config binds
#define HIGH_SPEED      1
#define BAUD            9600
#define FOSC            4000000
#define PIC_CLK         4000000
#define _XTAL_FREQ      4000000
#define NINE_BITS       0
#define SPEED           0x4
#define RX_PIN          TRISC5
#define TX_PIN          TRISC4
#define DIVIDER         ((int)(FOSC/(16UL * BAUD) -1))

// ADC config binds
#define ADC_select      ADCON0 = 0b00011100
#define ADC             ADCON0 = 0b00010000
// delay for current measurement
#define ADC_delay       30
// thresholds current measurement
#define no_current      30
#define ok_current      60
#define over_current    78


// Led's config binds
#define LED_RED         PORTAbits.RA4
#define LED_GREN        PORTAbits.RA5

// ON/OFF config binds
#define on_off          PORTCbits.RC2
#define ground          PORTCbits.RC1

// Built-in delay routine
#pragma inline(_delay)
#define __delay_us(x)   _delay((unsigned long)((x)*(_XTAL_FREQ/4000000.0)))
#define __delay_ms(x)   _delay((unsigned long)((x)*(_XTAL_FREQ/4000.0)))


unsigned char adc_adr;
unsigned char current;
unsigned char adresstable(unsigned char adr);
unsigned char tmp = 0;
unsigned char alarm = 0x00;


__CONFIG(FOSC_INTOSC & WDTE_OFF & PWRTE_OFF & MCLRE_OFF & CP_OFF & BOREN_OFF & CLKOUTEN_OFF & IESO_OFF & FCMEN_OFF);
__CONFIG(WRT_OFF & PLLEN_OFF & STVREN_OFF & BORV_19 & LVP_OFF);

void usart_init(void);
void usart_putch(unsigned char c);
extern void _delay(unsigned long);

union pack_in_t { //Incoming packages
    unsigned char byte;

    struct {
        unsigned char cmd : 4; //Command addressed to device (4-bits)
        unsigned char dev : 4; //Address of the recipient (4-bits)
    };
} pack_in;

union pack_out_t { //Response to the device to the sender
    unsigned char byte;

    struct {
        unsigned char current : 2;
        unsigned char ground_key : 1;
        unsigned char power_key : 1;
        unsigned char dev : 4;
    };
} pack_out;

void usart_init(void) { //
    TRISC4 = 1;
    TRISC5 = 1;
    SPBRG = DIVIDER;
#if HIGH_SPEED == 1
    BRGH = 1;
#else
    BRGH = 0;
#endif
    SYNC = 0;
    SPEN = 1;
    TXIE = 0;
    TX9  = 0;
    RX9  = 0;
    TXEN = 1;
    CREN = 1;
    RCIE = 0;
    RCIF = 0;
}

unsigned char adresstable(unsigned char adc_adr) {
    if (adc_adr <= 23)  return 0b0000; //1-channel
    if (adc_adr <= 39)  return 0b0001; //2-channel
    if (adc_adr <= 55)  return 0b0010; //3-channel
    if (adc_adr <= 71)  return 0b0011; //4-channel
    if (adc_adr <= 87)  return 0b0100; //5-channel
    if (adc_adr <= 103) return 0b0101; //6-channel
    if (adc_adr <= 118) return 0b0110; //7-channel
    if (adc_adr <= 135) return 0b0111; //8-channel
    if (adc_adr <= 151) return 0b1000; //9-channel
    if (adc_adr <= 167) return 0b1001; //10-channel
    if (adc_adr <= 183) return 0b1010; //11-channel
    if (adc_adr <= 199) return 0b1011; //12-channel
    if (adc_adr <= 215) return 0b1100; //13-channel
    if (adc_adr <= 231) return 0b1101; //14-channel
    if (adc_adr <= 247) return 0b1110; //15-channel
    if (adc_adr <= 255) return 0b1111; //16-channel
}

unsigned char current_table(unsigned char current) {
    if (current <= no_current)   return 0b00; //No current in channel
    if (current <= ok_current)   return 0b01; //OK
    if (current <= over_current) return 0b11; //0b10 == 0b11 == overcurrent
    return 0b11;
}

void achtung(void) { //Disable the device when overcurrent
    ground = 1;
    on_off = 0;
    LED_GREN = 0;
    LED_RED = 1;
    pack_out.power_key = 0b0;
    pack_out.ground_key = 0b1;
    pack_out.current = 0b11;

    if (alarm != 0xBB) {
        eeprom_write(0xBB, 0xBB);
        alarm = 0xBB;
    }
}


unsigned char get_current(void) { //Handling the current sensor
    if (alarm != 0xBB){
        ADC;
        ADON = 1;
        __delay_us(ADC_delay);
        ADGO = 1;
        while (ADGO == 1) {
        }
        current = ADRESH;
        ADON = 0;
        if (current >= over_current) {
           achtung();
        }
        else{
           pack_out.current = current_table(current);
        }
}
    else{
        pack_out.current = 0b11;
    }


}

//ADDRESSING DEVICES AND THEIR CHANNELS
void comand(void) {
    switch (pack_in.cmd) {

        case 0b0000: //Status
            pack_out.power_key = on_off;
            pack_out.ground_key = ground;
            break;

        case 0b0001://ON channel
            if (on_off == 0) {
                ground = 0;
                LED_GREN = 1;
                on_off = 1;
                pack_out.power_key = 0b1;
                pack_out.ground_key = 0b0;
                if (alarm == 0xBB) {
                    alarm = 0x00;
                    LED_RED = 0;
                    eeprom_write(0xBB, 0x00);
                }
            }
            break;

        case 0b0010: //OFF channel
            if (on_off == 1) {
                on_off = 0;
                if (ground == 0) {
                    ground = 1;
                }
            }
            pack_out.power_key = 0b0;
            pack_out.ground_key = 0b1;
            LED_GREN = 0;
            if (alarm == 0xBB) {
                alarm = 0x00;
                LED_RED = 0;
                eeprom_write(0xBB, 0x00);
            }
            break;

        case 0b0011: // Impulse 5ms
            if (on_off == 1 && alarm != 0xBB) {
                LED_RED = 1;
                ground = 1;
                __delay_ms(5);
                ground = 0;
                LED_RED = 0;
                pack_out.power_key = 0b1;
                pack_out.ground_key = 0b1;
            }
            break;

        case 0b0100: // Impulse 10ms
            if (on_off == 1 && alarm != 0xBB) {
                LED_RED = 1;
                ground = 1;
                __delay_ms(10);
                ground = 0;
                LED_RED = 0;
                pack_out.power_key = 0b1;
                pack_out.ground_key = 0b1;
            }
            break;

        case 0b0101: // Impulse 15ms
            if (on_off == 1 && alarm != 0xBB) {
                LED_RED = 1;
                ground = 1;
                __delay_ms(15);
                ground = 0;
                LED_RED = 0;
                pack_out.power_key = 0b1;
                pack_out.ground_key = 0b1;
            }
            break;

        case 0b0110: // Impulse 30ms
            if (on_off == 1 && alarm != 0xBB) {
                LED_RED = 1;
                ground = 1;
                __delay_ms(30);
                ground = 0;
                LED_RED = 0;
                pack_out.power_key = 0b1;
                pack_out.ground_key = 0b1;
            }
            break;

        case 0b0111: // Impulse 50ms
            if (on_off == 1 && alarm != 0xBB) {
                LED_RED = 1;
                ground = 1;
                __delay_ms(50);
                ground = 0;
                LED_RED = 0;
                pack_out.power_key = 0b1;
                pack_out.ground_key = 0b1;
            }
            break;

        case 0b1000: // Impulse 80ms
            if (on_off == 1 && alarm != 0xBB) {
                LED_RED = 1;
                ground = 1;
                __delay_ms(80);
                ground = 0;
                LED_RED = 0;
                pack_out.power_key = 0b1;
                pack_out.ground_key = 0b1;
            }
            break;

        default:
            pack_out.power_key = on_off;
            pack_out.ground_key = ground;
            break;
    }

get_current();
putch_out_byte();

}

unsigned char get_adc_adress() { //Getting the address of the device through the ADC
    ADC_select;
    ADON = 1;
    __delay_us(ADC_delay);
    ADGO = 1;
    while (ADGO == 1) {
    }
    adc_adr = ADRESH;
    ADON = 0;
    tmp = adresstable(adc_adr);
    pack_out.dev = tmp;
    eeprom_write(0x02, tmp);
    eeprom_write(0x04, 0xAA);
//    putch_out_byte();   

}

putch_out_byte() { //Send a reply head unit
    while (!TXIF)
        continue;
    TXREG = pack_out.byte;
}

unsigned char getch() { //Getting UART

    return RCREG;

}

void main(void) {

    OSCTUNE = 0b00000000;
    OSCCON = 0b01101010;
    TRISA = 0b11001111;
    TRISC = 0b11111001;
    ground = 1;
    on_off = 0;
    pack_out.power_key = 0b0;
    pack_out.ground_key = 0b1;
    LED_GREN = 1;
    __delay_ms(600);
    LED_GREN = 0;
    INTCON = 0;
    ANSELC = 0b00001001;
    ADCON1 = 0b01010000;
    unsigned char myaddr;
    unsigned char no_address = 0;
    
    if (eeprom_read(0xBB) == 0xBB) {// Marking a broken device startup
        alarm = 0xBB;
        achtung();
    }

    if (eeprom_read(0x04) == 0xAA) {// Device have address in EEPROM, get it from the ADC and remember
        myaddr = eeprom_read(0x02);
    } else {
        __delay_us(1000);
        LED_RED = 1;
        LED_GREN = 1;
        no_address = 1;
    }

    usart_init(); // UART initialization

    while (1) {

        if (RCIF == 1) {

            pack_in.byte = getch(); //Pack 8-bit package to structure

            if (pack_in.byte == 0b00001111) { //Service comand for inquiry of addresses (Broadcast)
                on_off = 0;
                ground = 1;
                LED_RED = 0;
                LED_GREN = 0;
                pack_out.power_key = 0;
                pack_out.ground_key = 1;
                get_current();
                get_adc_adress(); //Command to set the new address
                myaddr = eeprom_read(0x02);
                no_address = 0;

            } else {
                if (no_address == 0) { //If the device already has an address
                    if (pack_in.dev == myaddr) {
                        pack_out.dev = myaddr;
                        comand();

                    }
                }
            }
           
            RCIF = 0;

        }
        get_current();
        
        }
    }

