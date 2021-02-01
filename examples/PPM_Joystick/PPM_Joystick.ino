
/*
 * Compile with Digispark 16.5 MHz
 * 
 * Acts as a Joystick with 6 axis and 6 buttons, receiving PPM signal from an RC remote.
 * It struggles with USB interrupts, but filters out PPM data when the USB interrupt fired inbetween.
 * For the other, successfully received data, it does an moving average filter to get rid of noise.
 * Causes a small latency, but wasn't able to feel it in DRL Simulator (ok, i am a newb so it might be due to this)
 * 
 * Hint: after refactoring parts of the library, this might not work properly.
 * 
 */
 
/* 
 * Error: multiple definition of `__vector_4'
 *
 *  As we have to use timer1 for overflow counting, disable the arduino milis() timer by editing
 *  <user directory>\AppData\Local\Arduino15\packages\digistump\hardware\avr\1.6.7\cores\tiny\wiring.c:92 
 *  
 *    ISR(MILLISTIMER_OVF_vect, ISR_NOBLOCK)   
 *  to
 *    void timer_cbr()     
 *     
 *  Then milis() won't work anymore, but we don't need it for this sketch.   
 *  Change it back after compiling.
*/
 
#include <avr/wdt.h>
#include "DigiJoystick_FPV.h"

#define RC_RECEIVER_PORT PB0
#define RISING_EDGE PINB & (1 << RC_RECEIVER_PORT)
#define CHANNELS 12
#define CLOCK_BITS (1 << CS12)

volatile uint32_t ppmValueMin[CHANNELS];
volatile uint32_t ppmValueMax[CHANNELS];
volatile uint16_t ppmValues[CHANNELS];
uint32_t avgValues[CHANNELS];
volatile uint16_t overflows = 0;
volatile uint16_t ppmValid = 0;
volatile uint8_t ppmPos = 0;

extern uint8_t usbActivityCount;

void setup()
{
  pinMode(PB0, INPUT);
  pinMode(PB1, OUTPUT);
  pinMode(PB2, INPUT);

  for(int pos = 0; pos < CHANNELS; pos++)
  {
    ppmValueMin[pos] = 1700;
    ppmValueMax[pos] = 2500;
    ppmValues[pos] = 0;
  }
  
  GIMSK |= 1<<PCIE;               // Enable pin change interrupts,
  PCMSK |= (1 << PCINT0);                  // Pin Change Enable Mask for PB0 (pin 5) (datasheet page 52)
  TCNT1 = 0;   
  TCCR1 = 0;
}


ISR(PCINT0_vect)
{
  if(RISING_EDGE)
  {
    TCNT1 = 0;                           // reset counter
    TIFR = (1 << TOV1);                  // clear timer interrupt overflow flag (NOTE: correct way to clear an interrupt; do NOT use "|=" )
    overflows = 0;                       // reset overflows;
    TCCR1 |= CLOCK_BITS;
    digitalWrite(1, LOW);
    return; 
  }
  
  TCCR1 &= ~CLOCK_BITS; 
  
  if(TIFR & (1 << TOV1))
  {
    overflows++;
  }
  
  uint16_t value = (overflows << 8) | (uint16_t)TCNT1;

  if(ppmPos < CHANNELS)
  {
    if(!usbActivityCount)
    {
      ppmValues[ppmPos] = value;
    }
    ppmPos++;
  }
    
  if(value > 4000)
  {
    digitalWrite(1, HIGH);
    ppmValid++;
    ppmPos = 0;
  }
  usbActivityCount = 0;
}

ISR(TIMER1_OVF_vect)
{
  overflows++;
  TIFR = (1 << TOV1);
}

byte getConverted(uint8_t pos, bool filtered = true)
{
  uint32_t value = filtered ? avgValues[pos] : ppmValues[pos];

  if(ppmValid > 10)
  {
    if(value < ppmValueMin[pos])
    {
      ppmValueMin[pos] -= 2;
    }
    if(value > ppmValueMax[pos])
    {
      ppmValueMax[pos] += 2;
    }
  }

  uint16_t minVal = ppmValueMin[pos];
  uint16_t maxVal = ppmValueMax[pos];

  if(minVal > value)
  {
    return (byte) 255;
  }
  
  value -= minVal;
  value *= 255;
  value /= (maxVal - minVal);

  if(value > 255)
  {
    return (byte) 255;
  }
  return (byte) value;
}

void loop()
{
  uint16_t buttons = 0;

  for(int pos = 0; pos < CHANNELS; pos++)
  {
    avgValues[pos] = (avgValues[pos] + ppmValues[pos]) / 2;
  }
  
  for(int pos = 0; pos < 6; pos++)
  {
    uint8_t value = getConverted(4+pos, false);
    uint16_t bitVal1 = 1<<(2*pos);
    uint16_t bitVal2 = 1<<(2*pos+1);

    if(value >= 0xC0)
    {
      buttons |= bitVal2;
    }
    else if(value >= 0x40)
    {
      buttons |= bitVal1;
    }
  }

  DigiJoystick.setX(getConverted(3));
  DigiJoystick.setY(getConverted(0));
  DigiJoystick.setXROT(getConverted(1));
  DigiJoystick.setYROT(getConverted(2));
  DigiJoystick.setZROT(getConverted(10));
  DigiJoystick.setSLIDER(getConverted(11));
  
  DigiJoystick.setButtons((byte) buttons, (byte)(buttons >> 8));

  DigiJoystick.update();
}

