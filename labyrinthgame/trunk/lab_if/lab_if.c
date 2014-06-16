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

/** oden der ligger på example.c .
output sættes ved at sende "F1=0/n" og "F1=1/n"
ligger på port F som følger
F0  Dir up/down
F1  Step up/down
F4  Dir left/right
F5  Step  left/right
F6  Magnet
F7  Analog input

input liger på B0, B1 og B2, og jeg mindes ikke hvilke der er hvilken,
men det er kranens rotations end stop, den optiske bro i kugle barken,
og så knappen for and på maskinen.
inputtet kan aflæses individuelt ved at sende "B0?/n"

der ud over kan man den n, o, r  og l, efter fult at tre cifre som så
er antallet af steps kranen skal tag henholdsvis ned, op, højre og
venstre. og mens den køre vil den ved hver step sende værdigen af B0,
B1 og B2 tilbage,  hvilke ikke er så brugbart µc'en ikke tag i mod
beskeder mens den køre

de to vinkler sættes med P og p  efter fult at tre cifre, P128  er
vandret eller tæt der på

okay jeg kan ikke lige finde  den fil jeg havde det stående i men jeg
kan huske at det var 1200 steps op.*/


#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include <util/delay.h>
#include "usb_serial.h"

#define LED_CONFIG	(DDRD |= (1<<6))
#define LED_ON		(PORTD |= (1<<6))
#define LED_OFF		(PORTD &= ~(1<<6))
#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))

void send_str(const char *s);
uint8_t recv_str(char *buf, uint8_t size);
void parse_and_execute_command(const char *buf, uint8_t num);
void InitPWM(void);
void SetPWMOutput0(uint8_t duty);
void SetPWMOutput1(uint8_t duty);
void SetPWMOutput(uint8_t duty, uint8_t port);
int localEcho = 1;
uint8_t timingChar = 'a';

#if 0
// Very simple character echo test
int main(void)
{
	CPU_PRESCALE(0);
	usb_init();
	while (1) {
		int n = usb_serial_getchar();
		if (n >= 0) usb_serial_putchar(n);
	}
}

#else



// Basic command interpreter for controlling port pins
int main(void)
{
  char buf[32];
  uint8_t n;
  const char * help = PSTR("\n"
    "Labyrinth USB interface (emborg/jca), " REV "\r\n"
    "Simple Control Shell\n\r\n"
    "Example Commands\r\n"
    "  B0?    Read Port B, pin 0\r\n"
    "  Oxxx   crane up    (xxx steps - max 500)\r\n"
    "  Nxxx   crane down  (xxx steps)\r\n"
    "  Lxxx   crane left  (xxx steps)\r\n"
    "  Rxxx   crane right (xxx steps)\r\n"
    "  Pxxx   PWM (xxx=value) tilt (x) - max 255\r\n"
    "  pxxx   PWM (xxx=value) roll (y)\r\n"
    "  F6=1   magnet (1=hold)\r\n"
    "  F7=1   enable roll/tilt (1=enable)\r\n"
    "  D6=1   Write to Port D pin 6 (D6 is LED pin)\r\n"
    "  B4=0   Get status for switches (pin B4 not used)\r\n"
    "  i=1    Interactive - use local echo of all commands\r\n"
    "  j=a    set timing char - is returned as part of status\r\n"
    "  help   This help text\r\n"
    "  All commands return switch status 'F000' [Ball Crane Start]\r\n"
    );

  // set for 16 MHz clock, and turn on the LED
  CPU_PRESCALE(0);
  LED_CONFIG;
  LED_ON;

// make the pin B0 B1 og B2 an input
  *(uint8_t *)(0x21 + 1 * 3) &= ~(1 << 0); // ball detect (0=ball)
  *(uint8_t *)(0x21 + 1 * 3) &= ~(1 << 1); // crane rotate switch (1=crane fully right (CCV))
  *(uint8_t *)(0x21 + 1 * 3) &= ~(1 << 2); // game start switch (0=pressed)
// make the pin F0-7  output
  *(uint8_t *)(0x21 + 5 * 3) |= ~(1 << 0);
  *(uint8_t *)(0x21 + 5 * 3) |= ~(1 << 1);
  *(uint8_t *)(0x21 + 5 * 3) |= ~(1 << 4);
  *(uint8_t *)(0x21 + 5 * 3) |= ~(1 << 5);
  *(uint8_t *)(0x21 + 5 * 3) |= ~(1 << 6);
  *(uint8_t *)(0x21 + 5 * 3) |= ~(1 << 7);
// sets pin F0-7  to 0
  *(uint8_t *)(0x22 + 5 * 3) &= ~(1 << 0); // crane lift direction
  *(uint8_t *)(0x22 + 5 * 3) &= ~(1 << 1); // crane lift step
  *(uint8_t *)(0x22 + 5 * 3) &= ~(1 << 4); // crane rotate direction
  *(uint8_t *)(0x22 + 5 * 3) &= ~(1 << 5); // crane rotate step
  *(uint8_t *)(0x22 + 5 * 3) &= ~(1 << 6); // magnet off
  *(uint8_t *)(0x22 + 5 * 3) &= ~(1 << 7); // set tilt/roll disable = 0

  // initialize the USB, and then wait for the host
  // to set configuration.  If the Teensy is powered
  // without a PC connected to the USB port, this
  // will wait forever.
  usb_init();
  while (!usb_configured())
    /* wait */ ;
  // and a bit more
  _delay_ms(1000);
  InitPWM();
  DDRD|=(1 << PD0);
  OCR0B=127;
  DDRB|=(1 << PB7);
  OCR0A=127;
  // main loop
  while (1)
  {
    // wait for the user to run their terminal emulator program
    // which sets DTR to indicate it is ready to receive.
    while (!(usb_serial_get_control() & USB_SERIAL_DTR)) /* wait */ ;

    // discard anything that was received prior.  Sometimes the
    // operating system or other software will send a modem
    // "AT command", which can still be buffered.
    usb_serial_flush_input();

    // print a nice welcome message
    send_str(help);

    // and then listen for commands and process them
    while (1)
    {
      if (localEcho)
        send_str(PSTR(">"));
      n = recv_str(buf, sizeof(buf));
      if (n == 255) 
        break;
      send_str(PSTR("\r\n"));   //\f
      if (buf[0] == 'h')
	send_str(help);
      else
        parse_and_execute_command(buf, n);
    }
  }
}
#endif

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
    if (!c) break;
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
  if (num < 3) 
  { // too short
    send_str(PSTR("unrecognized format, 3 chars min req'd\r\n"));
    return;
  }
  // first character is the port letter
  if (buf[0] >= 'A' && buf[0] <= 'F')
  {
    port = buf[0] - 'A';
  }
  else if (buf[0] >= 'a' && buf[0] <= 'f')
  {
    port = buf[0] - 'a';
  }
  else if(buf[0]=='P'||buf[0]=='p')
  { // PWM
    val=buf[1]-'0';
    val*=10;
    val=buf[2]-'0'+ val;
    if (num>=4)
    {
      val*=10;
      val=buf[3]-'0'+val;
    }
    if (buf[0]=='P')
    {
      SetPWMOutput0(val);
    }
    else
    {
      SetPWMOutput1(val);
    }
//     // send status as responce
//     usb_serial_putchar('F');
//     val2 = *(uint8_t *)(0x20 + 1 * 3) & (1 << 0);
//     usb_serial_putchar(val2 ? '1' : '0');
//     val2 = *(uint8_t *)(0x20 + 1 * 3) & (1 << 1);
//     usb_serial_putchar(val2 ? '1' : '0');
//     val2 = *(uint8_t *)(0x20 + 1 * 3) & (1 << 2);
//     usb_serial_putchar(val2 ? '1' : '0');
//     val2 = *(uint8_t *)(0x20 + 1 * 3) & (1 << 3);
//     usb_serial_putchar(val2 ? '1' : '0');
//     usb_serial_putchar(timingChar);
//     send_str(PSTR("\r\n"));
    isOK = 1;
  }
  else if(buf[0]=='N' || buf[0]=='n'||
          buf[0]=='O' || buf[0]=='o'||
          buf[0]=='R' || buf[0]=='r'||
          buf[0]=='L' || buf[0]=='l')
  {  // N=ned, O=op, R= højre, L=venster
    val=buf[1]-'0';
    val*=10;
    val=buf[2]-'0'+ val;
    if (num>=4)
    {
      val*=10;
      val=buf[3]-'0'+val;
    }
    port = 5 ;// port F
    port = 'F' - 'A';
    if(buf[0]=='N'||buf[0]=='n')
    {  // ned
      pin =0;
      *(uint8_t *)(0x21 + port * 3) |= (1 << pin);
      *(uint8_t *)(0x22 + port * 3) &= ~(1 << pin);

      *(uint8_t *)(0x21 + port * 3) |= (1 << pin);
      *(uint8_t *)(0x22 + port * 3) &= ~(1 << pin);
      pin=1;
    }
    if(buf[0]=='O'||buf[0]=='o')
    { // op
      pin =0;
      *(uint8_t *)(0x21 + port * 3) |= (1 << pin);
      *(uint8_t *)(0x22 + port * 3) |= (1 << pin);

      *(uint8_t *)(0x21 + port * 3) |= (1 << pin);
      *(uint8_t *)(0x22 + port * 3) |= (1 << pin);
      pin=1;
    }
    if(buf[0]=='R'||buf[0]=='r')
    {  // ned
      pin =4;
      *(uint8_t *)(0x21 + port * 3) |= (1 << pin);
      *(uint8_t *)(0x22 + port * 3) &= ~(1 << pin);
      pin=5;
    }
    if(buf[0]=='L'||buf[0]=='l')
    { // op
      pin =4;
      *(uint8_t *)(0x21 + port * 3) |= (1 << pin);
      *(uint8_t *)(0x22 + port * 3) |= (1 << pin);

      *(uint8_t *)(0x21 + port * 3) |= (1 << pin);
      *(uint8_t *)(0x22 + port * 3) |= (1 << pin);
      pin=5;
    }
    // make the pin B0 B1 og B2 an input
    *(uint8_t *)(0x21 + 1 * 3) &= ~(1 << 0);
    *(uint8_t *)(0x21 + 1 * 3) &= ~(1 << 1);
    *(uint8_t *)(0x21 + 1 * 3) &= ~(1 << 2);
    // read the pin
    if (val > 500)
      // limit repeat to max 500
      val = 500;
    for(uint16_t J = 0; J < val; J++)
    {
      *(uint8_t *)(0x21 + port * 3) |= (1 << pin);
      // drive it low
      *(uint8_t *)(0x22 + port * 3) &= ~(1 << pin);
      // send status
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

      _delay_ms(4);
      // make the pin an output
      *(uint8_t *)(0x21 + port * 3) |= (1 << pin);
      // drive it high
      *(uint8_t *)(0x22 + port * 3) |= (1 << pin);
      _delay_ms(4);
    }
    isOK = 1;
//    return;
    _delay_ms(1);
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
  else
  {
    send_str(PSTR("Unknown command \""));
    usb_serial_putchar(buf[0]);
    send_str(PSTR("\", must be A - F, O, N, L, R, i, j, p or P\r\n"));
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

void InitPWM()
{
  //Set 8-bit fast PWM mode
  TCCR0A|=(1<<WGM00)|(1<<WGM01)|(1<<CS00)|(1<<COM0A1)|(1<<COM0B1);
  TCCR0B|=(1<<CS00);
  //Initialize port D as output
  DDRB|=(1 << PB7);
  DDRD|=(1 << PD0);
  return;
}

void SetPWMOutput0(uint8_t duty)
{
   DDRB|=(1 << PB7);
   OCR0A=duty;
   return;
}
void SetPWMOutput1(uint8_t duty)
{
   DDRD|=(1 << PD0);
   OCR0B=duty;
   return;
}
void SetPWMOutput(uint8_t duty, uint8_t port)
{
     if (port==0){
                DDRB|=(1 << PB7);
                OCR0A=duty;
                return;
     }else if (port==1){
     DDRD|=(1 << PD0);
     OCR0B=duty;
     return;
     }else{
	send_str(PSTR("Unknown PWM channel \""));
    return;
    }
 }
