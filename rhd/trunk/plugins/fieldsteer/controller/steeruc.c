/* Simple example for Teensy USB Development Board
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2008 PJRC.COM, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#define REV "$Rev: 231 $"

/**
 * Controller for steering servo and magnetic encoders
 * for DTU field robot */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>
#include "usb_serial.h"
#include "dynamixel.h"
#include "dxl_hal.h"

#define LED_CONFIG  (DDRD  |= (1 << 6))
#define LED_ON      (PORTD |= (1 << 6))
#define LED_OFF     (PORTD &= ~(1 << 6))
#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))

const char * lfcr = "\n\r";
void send_str(const char *s);
uint8_t recv_str(char *buf, uint8_t size);
void parse_and_execute_command(const char *buf, uint8_t num);
// void InitPWM(void);
// void SetPWMOutput0(uint8_t duty);
// void SetPWMOutput1(uint8_t duty);
// void SetPWMOutput(uint8_t duty, uint8_t port);
int localEcho = 1;
uint8_t timingChar = 'a';

// Encoder (3 bytes for one (left) encoder 5 for two (left and right) and
// ? bytes buffer for 3 encoders (left, right and tilt))
#define EBL 3
// NB number of buffes fixed to 4 - change require code change too.
#define EBL_N 4
uint8_t encBuf[EBL_N][EBL];
uint8_t encBufCnt = 0;
uint8_t encBufN = 0;
uint8_t encBufOK[EBL_N] = {0,0,0,0};
uint16_t encCnt = 0;
#define ENC_MEDIAN_VALUES 5
//uint16_t encLlast, encRlast;
uint8_t  encLadd = 0, encRadd = 0;
uint16_t encL, encR, encT = 0; //, encLRaw[ENC_MEDIAN_VALUES], encRRaw[ENC_MEDIAN_VALUES];
uint16_t encTmp[ENC_MEDIAN_VALUES];
uint8_t /*encL_OCF, encL_COF, encL_LIN,*/ encL_MAG, encLhold = 0; //, encL_PAR;
uint8_t /*encR_OCF, encR_COF, encR_LIN,*/ encR_MAG, encRhold = 0; //, encR_PAR;
uint8_t encT_MAG = 0, encThold = 0;
//uint16_t encLParityErrCnt = 0, encRParityErrCnt = 0;

// timer state
uint8_t timerState = 0;
//uint8_t encCycleRead = 0;
uint8_t delayCnt;
uint16_t timerStateCnt = 0;
// timer A clock (CLK_IO / prescaler)
#define TIMER_A_CLOCK (16/1)
// encoder read time in us * CLK_IO frequency in MHz
#define ENCODER_READ_TIME_US_N 390
#define ENCODER_READ_CNT_N (ENCODER_READ_TIME_US_N * TIMER_A_CLOCK)
#define CYCLE_TIME_US 10000
int16_t remaining_cycleTime;
char s2[4] = "ab\0"; // debug
int8_t  remaining_cycles;
char s3[4] = "12\0"; // debug
int spiCnt = 0;
uint16_t timer1Cnta, timer1Cntb, timer1Cntc, timer1Cntd, timer1Cnte;
uint16_t timer1Cnt = 0;
#define MSTL 200
char s5[MSTL];
uint8_t dxlTalkOK = 0;
uint8_t space4dxlTalk = 12, space4dxlTalkCnt; // space for about 12 characters (tx+rx)
uint16_t dxlRxVal, dxlTXVal;
uint8_t dxlID;
uint8_t dxlAdr;
uint8_t dxlWord;
uint8_t dxlCmdTx = 0; // command valid (not send)
uint8_t dxlCmdRx = 0; // reply valid

/**
 * SPI data transfer complete interrupt (8 bit) */
ISR(SPI_STC_vect)
{ // save data from AS5045 magnetic encoder
  encBuf[encBufN][encBufCnt] = SPDR;
  encBufCnt++;
  if (encBufCnt < EBL)
  { // not all data received
    // start transfer of next byte
    SPDR = 0x3C;
  }
  else
  { // stop read
    PORTB |= (1 << PB0); // 0xfb;  // stop SPI, set SS to high
    // filled buffer is valid
    encBufOK[encBufN] = 1;
    encBufN = (encBufN + 1) & 0x3;
    // new buffer is now invalid and empty
    encBufOK[encBufN] = 0;
    encBufCnt = 0;
  }
  spiCnt++;
}

///
/** timer 1 (16 bit) interrupt
 * this is used to split time into encoder read and communication with servos
 * state 1 and 2 is reserved time for serco communication,
 * state 3 and 4 is encoder read states */
ISR(TIMER1_COMPA_vect)
{
  // increase timer for simplex receive timeout
  gwCountNum +=OCR1A;
  // debug timer
  timer1Cnt++;
  // do encoder read sycles
  switch (timerState)
  {
  case 1: // time to talk to servos
     OCR1A = gfByteTransTime_us/10 * TIMER_A_CLOCK; // wait on character
     dxlTalkOK = 1;
     PORTC |= 0x80;
     timer1Cnta = TCNT2;
     space4dxlTalkCnt = 0;
    break;
  case 2: // stop talking to servos
     space4dxlTalkCnt++;
     if (space4dxlTalkCnt < space4dxlTalk)
       // stay here one sycle still
       timerState--;
     else
     { // end txl talk time
       dxlTalkOK = 0;
       PORTC &= ~0x80;
     }
     timer1Cntb = TCNT2;
    break;
  case 3: // encoder read start
//    PORTD &= ~(1 << 5); // stop motor 2 pulse
    // calculate remaining cycle time - leave 0.5ms for pre-motor control
//     remaining_cycleTime = CYCLE_TIME_US - 500 -
//                           (motorLeftPWM /*/ TIMER_A_CLOCK*/) -
//                           (motorRightPWM /*/ TIMER_A_CLOCK*/);
    // remaining_cycles = remaining_cycleTime / ENCODER_READ_TIME_US_N;
    remaining_cycles = 5; //- should give 5 reads
    // wait a little longer first time to reach
    // a fixed cycle time
    OCR1A = ENCODER_READ_CNT_N /*+
            (remaining_cycleTime - remaining_cycles * ENCODER_READ_TIME_US_N) *
            TIMER_A_CLOCK*/;
    PORTB &= ~(1 << PB0); // 0xfb;  // start SPI, set SS to low
    PORTB &= ~(1 << PB0); // wait at least 0.5 us
    PORTB &= ~(1 << PB0); // wait at least 0.5 us
    SPDR = 0x3C; // start SPI read
    delayCnt = 1; // for reads in one cycle (approx 20 - depends on motor puls width)
    //cycleValueCnt = 0; // full read cycle - up to 4 in one cycle
    timer1Cntc = TCNT2;
    break;
  case 4: // encoder read
    OCR1A = ENCODER_READ_CNT_N; // ~400us sample rate for encoder 2kHz
    if (encBufCnt == 0)
    {  // start new read
      PORTB &= ~(1 << PB0); // 0xfb;  // start SPI, set SS to low
      PORTB &= ~(1 << PB0); // wait at least 0.5 us
      PORTB &= ~(1 << PB0); // wait at least 0.5 us
      SPDR = 0x3C;
    }
    delayCnt++;
    if (delayCnt < (remaining_cycles))
//    if (encCycleRead < (ENC_MEDIAN_VALUES - 1))
      // stay in this read mode for (5) reads
      timerState--;
    timer1Cntd = TCNT2;
    break;
//    case 5: // wait further before next cycle
//      delayCnt++;
//      if (delayCnt < remaining_cycles)
//        timerState--;
//      break;
  default: // finished - restart
    timerState = 0;
    OCR1A = 100 * TIMER_A_CLOCK; // ~1ms wait after encoder read
    timer1Cnte = TCNT2;
    break;
  }
  timerState++;
}

/**
 * Decode message from 1..3 AS5045 encoders received in one buffer (encBuf)
 * encoder value is stored in a arw encoder value buffer encXRaw, to
 * allow filtering of stary error values (buffer with 5 values).
 * OCF when 1: offset compensation finished is good
 * COF Cordic overflow - when 1:value is form last good measurement
 * LIN liniarity is bad, when 1: magnet alignmenet out of order
 * MAG when 0 magnets are good and stable, 1 is increase, 2 is decrease 3 is bad.
 * PAR is parity, when 1 parity is OK.
 * conts number of parity errors */
void encoderMsgDecode(uint8_t * buf)
{ // decode bytes from AS5045 magnetic encoder
  // left status bits
//   encL_OCF = (buf[1] >> 2) & 0x01;
//   encL_COF = (buf[1] >> 1) & 0x01;
//   encL_LIN =  buf[1] & 0x01;
//   encL_MAG =  buf[2] >> 6;
  if (encLhold == 0)
  { // status
    // bit 5: !COF
    encL_MAG = (((~buf[1] & 0x04) + (buf[1] & 0x03)) << 2) + (buf[2] >> 6);
    if (encL_MAG != 0)
      encLhold = 30;
  }
  if (encLhold > 0)
    encLhold--;
//   encL_PAR = parity_even_bit(
//                parity_even_bit(buf[0] << 1) +
//                parity_even_bit(buf[1]) +
//                parity_even_bit(buf[2] >> 4));
  // right status bits
//   encR_OCF = (buf[4] >> 7) & 0x01;
//   encR_COF = (buf[4] >> 6) & 0x01;
//   encR_LIN = (buf[4] >> 5) & 0x01;
//   encR_MAG = (buf[4] >> 3) & 0x03;
  if (encRhold == 0)
  { // noerror last time
    encR_MAG = (((~buf[4] & 0x80) + (buf[4] & 0x78)) >> 3);
    if (encR_MAG != 0)
      encRhold = 30;
  }
  if (encRhold > 0)
    encRhold--;
  if (encThold == 0)
  { // noerror last time
    encT_MAG = (((~buf[4] & 0x80) + (buf[4] & 0x78)) >> 3);
    if (encT_MAG != 0)
      // error hold error for some time
      encThold = 30;
  }
  if (encThold > 0)
    encThold--;

//   encR_PAR = parity_even_bit(
//                parity_even_bit(buf[2] << 4) +
//                parity_even_bit(buf[3]) +
//                parity_even_bit(buf[4] >> 2));
//   if ((encL_PAR == 1) && (encR_PAR == 1))
  { // both are good, so extract encoder values
//    uint16_t enc;
    //int16_t di;
    // left encoder
    encL = ((buf[0] & 0x7f) << 5) + (buf[1] >> 3);
//    encL = enc;
//    encLRaw[encCycleRead] = (encLadd << 12) + enc;
    // right encoder
    encR = ((buf[2] & 0xf) << 8) + buf[3];
//    encR = enc;
//    encRRaw[encCycleRead] = (encRadd << 12) + enc;
//    if (encCycleRead < ENC_MEDIAN_VALUES)
//      encCycleRead++;
  }
//   else
//   {
//     if (encL_PAR != 1)
//       encLParityErrCnt++;
//     if (encR_PAR != 1)
//       encRParityErrCnt++;
//   }
}


/**
 * ADC interrupt routine */
ISR(ADC_vect)
{
  // start new conversion
  //ADCSRA = AD_START_CONV;
  sei();
}

/**
 * Send string - and wait until send
 * \param str is the string to send */
void sendString(const char * str)
{
  uint16_t n = strlen(str);
  usb_serial_write((uint8_t *) str, n);
//   while (1)
//   { // send one char at a time until no more
//     if (str[i] == '\0')
//       break;
//     serial_write(str[i++]);
//   }
}

void sendStringLn(const char * str)
{
  sendString(str);
  sendString(lfcr);
}

/**
 * Send status of control mode */
void sendCtrlStatus(void)
{
//   const int MSL = 20;
//   char s[MSL];
  if (PORTD & (1 << PD6))
    PORTD &= ~(1 << PD6);
  else
    PORTD |=  (1 << PD6);
  snprintf(s5, MSTL, "E%3x,%3x,%3x", encL, encR, encT);
  sendString(s5);
  snprintf(s5, MSTL, "M%2u,%2u,%2u", encL_MAG, encR_MAG, encT_MAG);
  sendString(s5);
//   snprintf(s5, MSTL, "#%5u %5d", remaining_cycleTime, remaining_cycles);
//   sendString(s5);
  snprintf(s5, MSTL, "~%u %u", spiCnt, timer1Cnt);
  sendStringLn(s5);
}

//==================================================
//Core functions
//==================================================
//Function: ioinit
//Purpose:      Initialize AVR I/O, UART and Interrupts
//Inputs:       None
//Outputs:      None
void ioinit(void)
{ // SPI setup
  //1 = output, 0 = input
  /* Set MOSI (PB2) and SCK (PB1) and SS (PB0) as output, all others input */
  DDRB = (1 << PB2) | (1<<PB1) | (1 << PB0);
  PORTB |= (1 << PB3);   //Enable pull-up on MISO pin
  PORTB |= (1 << PB0); // set SS to 1 (tell slaves to disable)
  // SPI interrupt, SPI as master, set SPI clock rate fck/64
  SPCR = (1 << SPIE) | (1 << SPE) | (1 << MSTR) | (1 << CPOL) | (1 << SPR1);
  SPSR = 0; // not double speed (bit 0 writable only)


  DDRD  = (1 << PD3) | (1 << PD6);  //Set Port D bit 3 (TX) and bit 6 (LED) to outputs - all other input
  PORTD = (1 << PD2);  //Enable the pull-up on Rx (not needed, we have an external)

#define BAUD57600 34
  dxl_hal_open(BAUD57600, 57600);
  // USART Baud rate: actually dynamexal is 57142 bit/sec, and
  // this fits with double speed (With 16 MHz Clock) and UBRR=34
//   UBRR1H = (BAUD57600 >> 8) & 0x7F;   //Make sure highest bit(URSEL) is 0 indicating we are writing to UBRRH
//   UBRR1L =  BAUD57600;
//   //Double the UART Speed
//   UCSR1A = (1 << U2X1);
//   //Enable Rx and Tx in UART and Rx interrupt
//   UCSR1B = (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1);
//   UCSR1C = (1 << UCSZ10) | (1 << UCSZ11);  //8-Bit Characters
  //  stdout = &mystdout; //Required for printf init

  //
  // Init timer 2
  //Set Prescaler to 8. (Timer Frequency set to 16Mhz)
  //Used for time calculation
//   TCCR2A = 0;   // no PWM, no output
//   TCCR2B = 0x7; // Divde 16MHz clock by 1024 to give 64us each count (max is then 16.3 ms)
//   TCNT2  = 0;   // reset counter to 0
//   TIMSK2 = (1 << TOIE2); // timer overflow interrupt (16 ms)

  //Init Timer 1 for CTC mode
   TCCR1A = 0; //Set no output pin effect
   TCCR1B = (1 << WGM12) | (1 << CS10);  //Set CTC mode, clk_IO/1 prescaler
   OCR1A = 0xfffe; // initial value
   TIMSK1 = (1 << OCIE1A); // on compare A (timer reset)

  //Initialize the ADC for Reads to control motor shut-down in case of over current
//   ADMUX = 0;    //Set the ADC channel to 0.
//   ADCSRA = (1<<ADEN)|(1<<ADATE)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); //Enable the ADC, auto-triggering, the ADC Interrupt and set the ADC prescaler to 64                                                                                                                                    //ADC clock = 250kHz
//   ADCSRA |= (1<<ADSC);  //Start the first conversion

  // make the pin C0..C4 an output used to control servo tack direction
  // and pin C7 is debug signal for dxl talk (no encoder read)
  DDRC |= ((1 << PINC0) || (1 << PINC1) || (1 << PINC2) || (1 << PINC3) || (1 << PINC7));
}


// Basic command interpreter for controlling port pins
int main(void)
{
#define BUF_SIZE 32
  char buf[BUF_SIZE];
  uint8_t n;

  // set for 16 MHz clock, and turn on the LED
  CPU_PRESCALE(0);
  LED_CONFIG; // D6 as output (LED)
  LED_ON;     // turn LED on


  ioinit();
  // initialize the USB, and then wait for the host
  // to set configuration.  If the Teensy is powered
  // without a PC connected to the USB port, this
  // will wait forever.
  usb_init();
  while (!usb_configured())
    /* wait */ ;
  // and a bit more
  _delay_ms(1000);
//  InitPWM();
//  DDRD  |= (1 << PD0);
//  DDRB  |= (1 << PB7);
//   OCR0B  = 127;
//   OCR0A  = 127;
  // main loop
  while (1)
  { // wait for the USB user to run
    // which sets DTR to indicate it is ready to receive.
    while (!(usb_serial_get_control() & USB_SERIAL_DTR)) /* wait */ ;

    // discard anything that was received prior.  Sometimes the
    // operating system or other software will send a modem
    // "AT command", which can still be buffered.
    usb_serial_flush_input();

    // print a nice welcome message
    send_str(PSTR("Field robot steering USB interface (jca), " REV "\r\n"));
    n = 0;
    // and then listen for commands and process them
    while (1)
    { // get index to potentially filled buffer
      uint8_t encb = (encBufN - 1) & 0x3;
      uint8_t m;
      // encoder PCI interface
      if (encBufOK[encb])
      { // new encoder message is available
        encoderMsgDecode(encBuf[encb]);
        encBufOK[encb] = 0; // buffer used
      }
      // send data to servo
      if (dxlCmdTx == 1 && dxlTalkOK)
      { // command waiting a value
        uint16_t got;
        if (dxlWord)
          got = dxl_read_byte(dxlID, dxlAdr);
        else
          got = dxl_read_word(dxlID, dxlAdr);
        // is send
        snprintf(s5, MSTL, "dxl id %2d, adr %2d=%d ok=%d", dxlID, dxlAdr, got, dxl_get_result());
        sendStringLn(s5);
        dxlCmdTx = 0;
        dxlCmdRx = 1; // not really needed
      }
      // get number of available chars
      m = usb_serial_available();
      if (m > 0)
      { // something on the USB channell
        buf[n] = usb_serial_getchar();
        buf[n + 1] = '\0';
        if (buf[n] == '\n' || buf[n] == '\r')
        { // there is a command terminated with either
          if (localEcho)
            send_str(PSTR("\r\n"));   //\f
          parse_and_execute_command(buf, n);
          if (localEcho)
            send_str(PSTR(">"));
          // clear command buffer
          n = 0;
          buf[n] = '\0';
        }
        if (buf[n] >= ' ' && buf[n] <= '~')
          n++;
        if (n >= BUF_SIZE)
          // input error - restart
          n = 0;
      }
    }
  }
}

// Send a string to the USB serial port.  The string must be in
// flash memory, using PSTR
//
void send_str(const char *s)
{
//   int n;
//   const char * p1 = s;
//   while (*p1)
//     p1++;
//   n = p1 - s;
//   usb_serial_write(s, n);
//   usb_serial_flush_output();
  char c;
  while (1)
  {
    c = pgm_read_byte(s++);
    if (!c)
      break;
    usb_serial_putchar(c);
  }
}

// Receive a string from the USB serial port.  The string is stored
// in the buffer and this function will not exceed the buffer size.
// A carriage return or newline completes the string, and is not
// stored into the buffer.
// The return value is the number of characters received, or 255 if
// the virtual serial connection was closed while waiting.
//
uint8_t recv_str(char *buf, uint8_t size)
{
  int16_t r;
  uint8_t count=0;

  while (count < size)
  {
    r = usb_serial_getchar();
    if (r != -1)
    {
      if (r == '\r' || r == '\n')
        return count;
      if (r >= ' ' && r <= '~')
      { // save character in buffer
        *buf++ = r;
        if (localEcho)
          // do we want an echo of what we send?
          usb_serial_putchar(r);
        count++;
      }
    }
    else
    {
      if (!usb_configured() ||
          !(usb_serial_get_control() & USB_SERIAL_DTR))
      { // user no longer connected
        return 255;
      }
      // just a normal timeout, keep waiting
    }
  }
  return count;
}

// parse a user command and execute it, or print an error message
//
void parse_and_execute_command(const char *buf, uint8_t num)
{
  uint8_t port = 0, pin = 0, val2;
  uint16_t val;
  char isOK = 0;
  const char * help = PSTR("\n"
    "Field robot steering USB interface (jca), " REV "\r\n"
    "Control Shell\n\r\n"
    "  F6=1   Write 1 to port pin F6\r\n"
    "  D6=1   Write 1 Port D pin 6 (D6 is LED pin)\r\n"
    "  B4=0   Get status for switches (pin B4 not used)\r\n"
    "  i=1    Interactive - use local echo of all commands\r\n"
    "  j=a    set timing char - is returned as part of status\r\n"
    "  r=a,b  dxl read byte from servo a at address (decimal)\r\n"
    "  R=a,b  dxl read word from servo a at address (decimal)\r\n"
    "  w=a,b,c  dxl write byte to servo a at address b to value c\r\n"
    "  W=a,b,c  dxl write word to servo a at address b to value c\r\n"
    "  s      Send control status\r\n"
    "  help   This help text\r\n"
    "  All commands return switch status 'F000' [Ball Crane Start]\r\n"
    );
//   if (num < 3)
//   { // too short
//     send_str(PSTR("unrecognized format, 3 chars min req'd\r\n"));
//     return;
//   }
  // first character is the port letter
  if (buf[0] >= 'A' && buf[0] <= 'F')
  {
    port = buf[0] - 'A';
  }
  else if (buf[0] >= 'a' && buf[0] <= 'f')
  {
    port = buf[0] - 'a';
  }
  else if (buf[0] == 'i' && buf[1] == '=')
  {
    localEcho = buf[2] == '1';
    isOK = 1;
  }
  else if (buf[0] == 'j' && buf[1] == '=')
  {
    timingChar = buf[2];
    isOK = 1;
  }
  else if (buf[0] == 's')
  {
    sendCtrlStatus();
    isOK = 1;
  }
  else if (buf[0] == 'h' || buf[0] == 'H')
  {
    send_str(help);
    isOK = 1;
  }
  else if ((buf[0] == 'r' || buf[0] == 'R') && buf[1] == '=')
  { // read operation (byte or word)
    char *p1;
    p1 = (char*)&buf[2];
    uint8_t id = strtol(p1, &p1,0);
    isOK = (*p1++ == ',');
    if (isOK)
    { // valid request - set request
      dxlCmdRx = 0;
      dxlCmdTx = 0;
      dxlID = id;
      dxlAdr = strtol(p1, &p1, 0);
      dxlWord = buf[0] == 'R';
      dxlRxVal = 0;
      // request is complete
      dxlCmdTx = 1;
    }
  }
  else
  {
    send_str(PSTR("Unknown command \""));
    usb_serial_putchar(buf[0]);
    send_str(PSTR("\", must be A - F, i, j\r\n"));
    isOK = 1;
  }
  // second character is the pin number
  if (isOK == 0)
  { // not handled port command (set or get)
    if (buf[1] >= '0' && buf[1] <= '7')
    {
      pin = buf[1] - '0';
    }
    else
    {
      send_str(PSTR("Unknown pin \""));
      usb_serial_putchar(buf[0]);
      send_str(PSTR("\", must be 0 to 7\r\n"));
      isOK = 2;
    }
    // if the third character is a question mark, read the pin
    if (isOK==0 && buf[2] == '?')
    { // make the pin an input
      *(uint8_t *)(0x21 + port * 3) &= ~(1 << pin);
      // read the pin
      val = *(uint8_t *)(0x20 + port * 3) & (1 << pin);
      usb_serial_putchar(val ? '1' : '0');
      send_str(PSTR("\r\n"));
      isOK = 2;
  //    return;
    }
    else
    { // not a read
      // if the third character is an equals sign, write the pin
      if (num >= 4 && buf[2] == '=')
      {
        if (buf[3] == '0')
        { // make the pin an output
          *(uint8_t *)(0x21 + port * 3) |= (1 << pin);
          // drive it low
          *(uint8_t *)(0x22 + port * 3) &= ~(1 << pin);
          isOK = 1;
  //        return;
        }
        else if (buf[3] == '1')
        { // make the pin an output
          *(uint8_t *)(0x21 + port * 3) |= (1 << pin);
          // drive it high
          *(uint8_t *)(0x22 + port * 3) |= (1 << pin);
          isOK = 1;
  //        return;
        }
        else
        {
          send_str(PSTR("Unknown value \""));
          usb_serial_putchar(buf[3]);
          send_str(PSTR("\", must be 0 or 1\r\n"));
          isOK = 2;
        }
      }
    }
  }
  // otherwise, error message
  if (isOK == 0)
  {
    send_str(PSTR("Unknown '"));
    usb_serial_putchar(buf[0]);
    send_str(PSTR("' command, must have ? or =\r\n"));
  }
  else if (isOK == 1)
  {
    // send status as responce
    usb_serial_putchar('F');
    val2 = *(uint8_t *)(0x20 + 1 * 3) & (1 << 0);
    usb_serial_putchar(val2 ? '1' : '0');
    val2 = *(uint8_t *)(0x20 + 1 * 3) & (1 << 1);
    usb_serial_putchar(val2 ? '1' : '0');
    val2 = *(uint8_t *)(0x20 + 1 * 3) & (1 << 2);
    usb_serial_putchar(val2 ? '1' : '0');
    val2 = *(uint8_t *)(0x20 + 1 * 3) & (1 << 3);
    usb_serial_putchar(val2 ? '1' : '0');
    usb_serial_putchar(timingChar);
    send_str(PSTR("\r\n"));
    usb_serial_flush_output();
  }
}

// void InitPWM()
// {
//   //Set 8-bit fast PWM mode
//   TCCR0A|=(1<<WGM00)|(1<<WGM01)|(1<<CS00)|(1<<COM0A1)|(1<<COM0B1);
//   TCCR0B|=(1<<CS00);
//   //Initialize port D as output
//   DDRB|=(1 << PB7);
//   DDRD|=(1 << PD0);
//   return;
// }
//
// void SetPWMOutput0(uint8_t duty)
// {
//    DDRB|=(1 << PB7);
//    OCR0A=duty;
//    return;
// }
// void SetPWMOutput1(uint8_t duty)
// {
//    DDRD|=(1 << PD0);
//    OCR0B=duty;
//    return;
// }
// void SetPWMOutput(uint8_t duty, uint8_t port)
// {
//      if (port==0){
//                 DDRB|=(1 << PB7);
//                 OCR0A=duty;
//                 return;
//      }else if (port==1){
//      DDRD|=(1 << PD0);
//      OCR0B=duty;
//      return;
//      }else{
// 	send_str(PSTR("Unknown PWM channel \""));
//     return;
//     }
//  }
