
/*
 * Compile with Digispark 16.5 MHz
 */
#include <avr/wdt.h>
#include "DigiJoystick_FPV.h"

#define RC_RECEIVER_PORT   PB0
#define CRSF_HIGH          (PINB & (1 << RC_RECEIVER_PORT))
#define CHANNELS           12
#define CLOCK_BITS         (1 << CS10)

/* just for reference, peripheral usage was actually troublesome */
#define BAUD_RATE          420000

/*
  from https://github.com/betaflight/betaflight/pull/1662/files
  0x14 Link statistics
  Uplink is the connection from the ground to the UAV and downlink the opposite direction.
  Payload:
  uint8_t     UplinkRSSI Ant.1(dBm*­1)
  uint8_t     UplinkRSSI Ant.2(dBm*­1)
  uint8_t     Uplink Package success rate / Link quality ( % )
  int8_t      Uplink SNR ( db )
  uint8_t     Diversity active antenna ( enum ant. 1 = 0, ant. 2 )
  uint8_t     RF Mode ( enum 4fps = 0 , 50fps, 150hz)
  uint8_t     Uplink TX Power ( enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW )
  uint8_t     Downlink RSSI ( dBm * ­-1 )
  uint8_t     Downlink package success rate / Link quality ( % )
  int8_t      Downlink SNR ( db )
*/
#define RX_BUFFER_SIZE     (28)
volatile uint8_t *receiveBuffer = NULL;
volatile uint8_t receiveLength = 0;
volatile uint8_t validRcData = 0;
volatile uint8_t receivedRcData[RX_BUFFER_SIZE];
volatile uint8_t receivedLinkData[RX_BUFFER_SIZE];
uint8_t receivedDataTemp[RX_BUFFER_SIZE];
uint8_t report[16];


volatile uint16_t channelValues[CHANNELS];
volatile uint16_t framesReceived = 0;
volatile uint16_t lastFrameNumber = 0;

/* USB code will update this counter. used to check if an USB interrupt happened.
 * If so, we will withdraw our sampling result
 */
extern uint8_t usbActivityCount;


void setup()
{
  pinMode(PB0, INPUT_PULLUP);
  pinMode(PB1, OUTPUT);
  pinMode(PB2, INPUT);

  for(int pos = 0; pos < CHANNELS; pos++)
  {
    channelValues[pos] = 0;
  }

  /* enable USB interrupt */
  GIMSK |= 1<<PCIE;

  /* set LED */
  PORTB &= ~(1<<1); 
}


/* handcrafted delay routine to get the ~2.4us bit timing of CRSF  */
static void delay_bit()
{
    __asm__ __volatile__ (
        " nop\n"
        " nop\n"
        " nop\n"
        " nop\n"
        " nop\n"
        " nop\n"
        " nop\n"
        " nop\n"
        " nop\n"
        " nop\n"
        " nop\n"
        " nop\n"
        " nop\n"
        " nop\n"
    );
}

#define STATE_WAIT_SYNC     0
#define STATE_RECEIVE_LEN   1
#define STATE_RECEIVE_TYPE  2
#define STATE_RECEIVE       3

/* for some unclear reason, the PCINT0 handler has a latency of about 2.5 microseconds.
 * so the first code gets executed just when the first data bit starts.
 * according to attiny85 documentation, we should have some 4 cycles plus a few instructions
 * in the interrupt body code that set up some registers.
 * so i expeced only about 1 microsecond.
 */
ISR(PCINT0_vect)
{
  /* must keep this local data static to save some CPU cycles.
   * even small changes have severe impact on sampling timing and causes invalid reads.
   * so please do not change code here if you cannot verify timing using a scope.
   */
  static uint8_t receivedByte = 0;
  static uint8_t bitVal = 1;
  static uint8_t state = 0;
  static uint8_t bytePos = 0;
  
  /* disable USB interrupts while we are receiving */
  GIMSK &= ~(1<<INT0);

  usbActivityCount = 0;
  receivedByte = 0;
  bitVal = 1;
  
  /* sample the 8 data bits: start, 0-7 then stop
   * ___   _   _   _   _   ___
   *    |_| |_| |_| |_| |_|  
   *     S 0 1 2 3 4 5 6 7 S
   * 
   * where the LED should blink:
   * ___       _____
   *    |_____|     |___   Rx line
   *       _     _
   * _____| |___| |_____   LED
   * 
   */
  while(bitVal != 0)
  {
    uint8_t bitState = (CRSF_HIGH != 0);

    /* toggle LED for debugging purposes */
    PORTB &= ~(1<<1);
    receivedByte |= (bitState ? bitVal : 0);

    bitVal <<= 1;
    delay_bit();
    /* toggle LED for debugging purposes */
    PORTB |= (1<<1);
  }
  
  /* handle depending on current state */
  switch(state)
  {
    case STATE_WAIT_SYNC:
      /* did we receive the CRSF sync byte 0xC8? */
      if(receivedByte == 0xC8)
      {
        /* yeah, start at position zero and receive length */
        bytePos = 0;
        state = STATE_RECEIVE_LEN;
      }
      break;

    case STATE_RECEIVE_LEN:
      receiveLength = receivedByte;
      state = STATE_RECEIVE_TYPE;
      break;

    case STATE_RECEIVE_TYPE:
      /* does the length byte announce a CRSF_FRAMETYPE_LINK_STATISTICS (0x14) with 0x0C byte payload? */
      if(receivedByte == 0x14 && receiveLength == 0x0C)
      {
        receiveBuffer = receivedLinkData;
        state = STATE_RECEIVE;
        receiveBuffer[bytePos++] = receivedByte;
      }
      /* does the length byte announce a CRSF_FRAMETYPE_RC_CHANNELS_PACKED (0x16) with 0x18 byte payload? */
      else if(receivedByte == 0x16 && receiveLength == 0x18)
      {
        /* still have received data? discard this new one as we do not know if there is copying/parsing in progress */
        if(validRcData)
        {
          state = STATE_WAIT_SYNC;
        }
        else
        {
          receiveBuffer = receivedRcData;
          state = STATE_RECEIVE;
          receiveBuffer[bytePos++] = receivedByte;
        }
      }
      else
      {
        /* if not, withdraw it and wait for sync */
        state = STATE_WAIT_SYNC;
      }
      break;

    case STATE_RECEIVE:
      receiveBuffer[bytePos++] = receivedByte;

      /* received [SYNC][LEN][type + payload + CRC-8] with a total of 26 bytes for RC channel frames */
      if(bytePos >= receiveLength)
      {
        if(receiveLength == 0x18)
        {
          validRcData = 1;
          framesReceived++;
        }
        state = STATE_WAIT_SYNC;
      }
      break;
  }

  /* if we are done receiving, enable USB interrupt again */
  if(state == STATE_WAIT_SYNC)
  {
    GIMSK |= (1<<INT0);
  }

  /* reset interrupt flags */
  GIFR |= (1<<PCIF) | (1<<INTF0);
}

byte getConverted(uint8_t pos)
{
  uint32_t value = channelValues[pos];
  return value >> 3;
}

uint8_t crc8 (uint8_t *addr, uint8_t len)
{
  uint8_t crc=0;
  for (uint8_t i=0; i<len;i++)
  {
    uint8_t inbyte = addr[i];
    for (uint8_t j=0;j<8;j++)
    {
      uint8_t mix = (crc ^ inbyte) & 0x80;
      crc <<= 1;
      if (mix)
        crc ^=  0xD5;
      inbyte <<= 1;
    }
  }
  return crc;
}


int state = 0;
uint64_t lastTransfer = 0;


void loop()
{
  uint64_t currentTime = millis();
  
  usbPoll();
  
  switch(state)
  {
    case 0:
      if(millis() > 1000)
      {
        state = 1;
      }
      return;

    case 1:
      /* normal state, handle CRSF interrupts */
      PCMSK |= (1 << PCINT0);
      
      if(validRcData)
      {
        memcpy(receivedDataTemp, (const void*)receivedRcData, 0x18);
        validRcData = 0;

        /* only handle packets with correct checksum */
        if(crc8(receivedDataTemp, 0x18) == 0)
        {
          uint8_t chanBits = 0;
          uint8_t channel = 0;
          uint32_t value = 0;

          /* go through all payload bytes */
          for(int pos = 0; pos < 22; pos++)
          {
            /* fetch 8 bits */
            value |= ((uint32_t)receivedDataTemp[1 + pos]) << chanBits; 
            chanBits += 8;

            /* when we got enough (11) bits, treat this as a sample */
            if(chanBits >= 11)
            {
              channelValues[channel++] = (value & 0x7FF);
              /* keep remaining bits */
              value >>= 11;
              chanBits -= 11;
            }
          }
          
          /* determine (3-state) buttons from channels 4 to 10 */
          uint16_t buttons = 0;
          for(int pos = 0; pos < 6; pos++)
          {
            uint8_t value = getConverted(4+pos);
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
/*
          report[0] = getConverted(0);
          report[1] = getConverted(1);
          report[2] = getConverted(2);
          report[3] = getConverted(3);
          report[4] = getConverted(10);
          report[5] = getConverted(11);
          report[6] = (byte)receivedLinkData[0];
          report[7] = (byte)receivedLinkData[1];
          report[8] = (byte)receivedLinkData[2];
          report[9] = (byte)receivedLinkData[3];
          report[10] = (byte)receivedLinkData[4];
          report[11] = (byte)receivedLinkData[5] << 4;
          report[12] = (byte)receivedLinkData[6] << 3;
          report[13] = (byte)receivedLinkData[7];
          */
          report[0] = getConverted(0);
          report[1] = getConverted(1);
          report[2] = getConverted(2);
          report[3] = getConverted(3);
          report[4] = getConverted(4);
          report[5] = getConverted(5);
          report[6] = getConverted(6);
          report[7] = getConverted(7);
          report[8] = getConverted(8);
          report[9] = getConverted(9);
          report[10] = getConverted(10);
          report[11] = getConverted(11);
          report[12] = (byte)receivedLinkData[6] << 3;
          report[13] = (byte)receivedLinkData[7];
          report[14] = (byte) (buttons);
          report[15] = (byte) (buttons >> 8);

          /* we built the frame to send, set INT data and disable CRSF reception */
          PCMSK &= ~(1 << PCINT0);
          usbSetInterrupt(report, 8);
          state = 2;
        }
      }

      /* make sure at least every 50ms a transfer happens */
      if(currentTime - lastTransfer > 50)
      {
        /* disable CRSF for now */
        PCMSK &= ~(1 << PCINT0);
        usbSetInterrupt(report, 8);
        state = 2;
      }
      break;

    case 2:
      /* queue second part of our HID data in the next interrupt */
      if(usbInterruptIsReady())
      {
        usbSetInterrupt(&report[8], 8);
        lastTransfer = currentTime;
        state = 3;
      }
      break;

    case 3:
      /* when interrupt buffer is empty, make sure we receive CRSF frames again */
      if(usbInterruptIsReady())
      {
        lastTransfer = currentTime;
        PCMSK |= (1 << PCINT0);
        state = 1;
      }
      break;
  } 
}











