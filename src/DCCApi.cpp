
#include "DCCWaveform.h"
#include "DCCApi.h"
#include "DIAG.h"
 
void DCCApi::begin() {
  DCCWaveform::begin();
}

void DCCApi::setSpeed(int loco, byte tSpeed, bool forward) {
  setSpeed(loco,tSpeed,forward,false);
}

 void DCCApi::setSpeed(int loco, byte tSpeed, bool forward, bool isReminder) {
  // create speed msg
  byte b[5];                      // save space for checksum byte
  byte nB = 0;

  if (loco > 127)
    b[nB++] = highByte(loco) | 0xC0;      // convert train number into a two-byte address

  b[nB++] = lowByte(loco);
  b[nB++] = 0x3F;                        // 128-step speed control byte
  b[nB++] = tSpeed + (tSpeed > 0) + forward * 128; // max speed is 126, but speed codes range from 2-127 (0=stop, 1=emergency stop)

   DCCWaveform::mainTrack.schedulePacket( b, nB,0);
  if (!isReminder) updateLocoReminder( loco,  tSpeed,  forward);
 }

   
void DCCApi::setFunction( int cab, int fByte, int eByte) 
{
 byte b[4];              
  byte nB = 0;

  if (cab>127)
    b[nB++] = highByte(cab) | 0xC0;      // convert train number into a two-byte address

  b[nB++] = lowByte(cab);

  if (eByte < 0) {                      // this is a request for functions FL,F1-F12  
    b[nB++] = (fByte | 0x80) & 0xBF;     // for safety this guarantees that first nibble of function byte will always be of binary form 10XX which should always be the case for FL,F1-F12  
  }
  else {                             // this is a request for functions F13-F28
    b[nB++] = (fByte | 0xDE) & 0xDF;     // for safety this guarantees that first byte will either be 0xDE (for F13-F20) or 0xDF (for F21-F28)
    b[nB++] = eByte;
  }

  /* NMRA DCC norm ask for two DCC packets instead of only one:
  "Command Stations that generate these packets, and which are not periodically refreshing these functions,
  must send at least two repetitions of these commands when any function state is changed."
  https://www.nmra.org/sites/default/files/s-9.2.1_2012_07.pdf
  */
    DCCWaveform::mainTrack.schedulePacket( b, nB,1);
 
} // RegisterList::setFunction(ints)


void DCCApi::setAccessory(int aAdd, int aNum, int activate)  
{
  byte b[2]; 

  b[0] = aAdd % 64 + 128;         // first byte is of the form 10AAAAAA, where AAAAAA represent 6 least significant bits of accessory address  
  b[1] = ((((aAdd / 64) % 8) << 4) + (aNum % 4 << 1) + activate % 2) ^ 0xF8;      // second byte is of the form 1AAACDDD, where C should be 1, and the least significant D represent activate/deactivate

   DCCWaveform::mainTrack.schedulePacket( b,2,0);

} 


  ///////////////////////////////////////////////////////////////////////////////

bool DCCApi::writeTextPacket( byte *b, int nBytes)  
{
  if (nBytes<2 || nBytes>5) return false;
   DCCWaveform::mainTrack.schedulePacket(b, nBytes,0);
  return true;
}


int DCCApi::readCV(int cv) 
{
  byte bRead[4];
  int bValue;

  cv--;                              // actual CV addresses are cv-1 (0-1023)


  bRead[0] = 0x78 + (highByte(cv) & 0x03);   // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
  bRead[1] = lowByte(cv);

  bValue = 0;

  for (int i = 0; i<8; i++) {
    bRead[2] = 0xE8 + i;
    DCCWaveform::progTrack.schedulePacket( resetMessage,2,2);          // NMRA recommends starting with 3 reset packets
    DCCWaveform::progTrack.schedulePacket(bRead, 3, 5);                // NMRA recommends 5 verify packets
    bool ret = DCCWaveform::progTrack.getAck();
    bitWrite(bValue, i, ret);
  }
  return verifyCV(cv,bValue)?bValue:-1;
}

 
///////////////////////////////////////////////////////////////////////////////

bool DCCApi::writeCVByte(int cv, int bValue) 
{
  byte bWrite[3];

  cv--;                              // actual CV addresses are cv-1 (0-1023)

  bWrite[0] = 0x7C + (highByte(cv) & 0x03);   // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
  bWrite[1] = lowByte(cv);
  bWrite[2] = bValue;

  DCCWaveform::progTrack.schedulePacket( resetMessage,2, 1);
  DCCWaveform::progTrack.schedulePacket(bWrite, 3, 4);
  
  return verifyCV(cv, bValue);
}  


bool DCCApi::writeCVBit(int cv, int bNum, int bValue) 
{
  byte bWrite[3];

  cv--;                              // actual CV addresses are cv-1 (0-1023)
  bValue = bValue % 2;
  bNum = bNum % 8;

  bWrite[0] = 0x78 + (highByte(cv) & 0x03);   // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
  bWrite[1] = lowByte(cv);
  bWrite[2] = 0xF0 + bValue * 8 + bNum;


  DCCWaveform::progTrack.schedulePacket(resetMessage,2,1);
  DCCWaveform::progTrack.schedulePacket(bWrite, 3, 4);

  bitClear(bWrite[2], 4);                    // change instruction code from Write Bit to Verify Bit
  DCCWaveform::progTrack.schedulePacket(resetMessage, 2,2);          // NMRA recommends starting with 3 reset packets
  DCCWaveform::progTrack.schedulePacket(bWrite, 3, 4);               // NMRA recommends 5 verfy packets
  return DCCWaveform::progTrack.getAck();
} 


///////////////////////////////////////////////////////////////////////////////

void DCCApi::writeCVByteMain(int cab, int cv, int bValue) 
{
  byte b[6];                     
  byte nB = 0;

  cv--;

  if (cab>127)
    b[nB++] = highByte(cab) | 0xC0;      // convert train number into a two-byte address

  b[nB++] = lowByte(cab);
  b[nB++] = 0xEC + (highByte(cv) & 0x03);   // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
  b[nB++] = lowByte(cv);
  b[nB++] = bValue;

  DCCWaveform::mainTrack.schedulePacket(b, nB, 4);

} 

void DCCApi::writeCVBitMain(int cab, int cv, int bNum, int bValue)  
{
  byte b[6];                     
  byte nB = 0;

  cv--;

  bValue = bValue % 2;
  bNum = bNum % 8;

  if (cab>127)
    b[nB++] = highByte(cab) | 0xC0;      // convert train number into a two-byte address

  b[nB++] = lowByte(cab);
  b[nB++] = 0xE8 + (highByte(cv) & 0x03);   // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
  b[nB++] = lowByte(cv);
  b[nB++] = 0xF0 + bValue * 8 + bNum;

   DCCWaveform::mainTrack.schedulePacket( b, nB, 4);
} 

////////////////////////////////////////////////////////// /////////////////////


void DCCApi::loop() {
   DCCWaveform::loop();
   // if the main track transmitter still has a pending packet, skip this loop.
  if ( DCCWaveform::mainTrack.packetPending) return;

  // each time around the Arduino loop, we resend a loco speed packet reminder 
  for (; nextLoco < MAX_LOCOS; nextLoco++) {
    if (speedTable[nextLoco].loco > 0) {
       setSpeed(speedTable[nextLoco].loco, speedTable[nextLoco].speed, speedTable[nextLoco].forward, true);
       nextLoco++;
      return;
    }
  }
  for (nextLoco = 0; nextLoco < MAX_LOCOS; nextLoco++) {
    if (speedTable[nextLoco].loco > 0) {
      setSpeed(speedTable[nextLoco].loco, speedTable[nextLoco].speed, speedTable[nextLoco].forward, true);
      nextLoco++;
      return;
    }
  }
 }


/// PRIVATE BELOW HERE 
bool DCCApi::verifyCV(int cv, byte bValue) {

  byte bRead[4];
  bRead[0] = 0x74 + (highByte(cv) & 0x03);   // set-up to re-verify entire byte
  bRead[2] = bValue;

  DCCWaveform::progTrack.schedulePacket(resetMessage, 2,2);          // NMRA recommends starting with 3 reset packets
  DCCWaveform::progTrack.schedulePacket(bRead, 3, 5);                // NMRA recommends 5 verify packets 
  return DCCWaveform::progTrack.getAck();
}

const byte idleMessage[]={0xFF,0x00};
const byte resetMessage[]={0x00,0x00};

byte DCCApi::nextLoco=0;  // position of loop in loco speed refresh cycle
DCCApi::LOCO DCCApi::speedTable[MAX_LOCOS];

 void  DCCApi::updateLocoReminder(int loco, byte tSpeed, bool forward) {
  // determine speed reg for this loco
  int reg;
  int firstEmpty=MAX_LOCOS;
  for (reg = 0; reg < MAX_LOCOS; reg++) {
    if (speedTable[reg].loco==loco) break;
    if (speedTable[reg].loco==0 && firstEmpty==MAX_LOCOS) firstEmpty=reg;
  }
  if (reg==MAX_LOCOS) reg=firstEmpty; 
  if (reg >= MAX_LOCOS) {
    DIAG(F("\nToo many locos\n"));
    return;
  }
  speedTable[reg].loco=loco;
  speedTable[reg].speed=tSpeed;
  speedTable[reg].forward=forward;
  nextLoco = reg;
  }