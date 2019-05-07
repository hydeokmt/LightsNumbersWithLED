#include <SPI.h>
#include <TimerOne.h>
#include "pins_arduino.h"

#define USE_SPI (1)
#define DIGIT_HIGH_BIT_NUM (8)
#define DIGIT_LOW_BIT_NUM (2)

const unsigned char toneSpkPin = 3;

#define SIZEOFARRAY(arr) (sizeof(arr) / sizeof(arr[0]))
const int writePortId[] = {A0, A1, A2, A3};
const int readPortId[] = {4, 5, 6, 7};
const char keyPadTable[SIZEOFARRAY(writePortId)][SIZEOFARRAY(readPortId)] = {
    {'1', '2', '3', ' '},
    {'4', '5', '6', ' '},
    {'7', '8', '9', ' '},
    {'0', ' ', ' ', '\r'}};
const int keyPadReadTimes = 5;
volatile unsigned char keyPadChange = 0;
volatile unsigned char button[SIZEOFARRAY(writePortId)][SIZEOFARRAY(readPortId)] = {
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0}};

const int firstBit = MSBFIRST;
// const int firstBit = LSBFIRST;
#if defined(USE_SPI)
#else  // defined (USE_SPI)
const int latchPin = 10; // 74HC595のST_CPへ
const int clockPin = 11; // 74HC595のSH_CPへ
const int dataPin = 13;  // 74HC595のDSへ
#endif // defined (USE_SPI)

char selectDigit;
unsigned long timeMicros;
unsigned long timeMillis;
class DigitPattern
{
  unsigned int **digitPatternArray;
  unsigned int digitPatternRow;
  unsigned int digitPatternCol;
  unsigned int *transferDigitPatternArray;
  unsigned int transferDigitPatternByte;
  unsigned char *transferBitPatternArray;
  unsigned int transferBitPatternByte;
  unsigned int transferBitPatternBit;
  typedef struct ControlBitPattern
  {
    unsigned char high; // ----rrrr
    unsigned char mid;  // rrrrrrrr
    unsigned char low;  // rr543210
  } ControlBitPattern;
  const ControlBitPattern controlBitPattern[2] = {
      // ----rrrr    rrrrrrrr    rr543210
      {0b00000000, 0b00000000, 0b00010101}, //0
      {0b00000000, 0b00000000, 0b00101010}, //1
  };
  typedef struct LedBitPattern
  {
    unsigned char high; // zyxgfedc
    unsigned char low;  // ba------
  } LedBitPattern;
  const LedBitPattern ledBitPattern[11] = {
      // zyxgfedc    ba------
      {0b00001111, 0b11000000}, //'0'
      {0b00000001, 0b10000000}, //'1'
      {0b00010110, 0b11000000}, //'2'
      {0b00010011, 0b11000000}, //'3'
      {0b00011001, 0b10000000}, //'4'
      {0b00011011, 0b01000000}, //'5'
      {0b00011111, 0b01000000}, //'6'
      {0b00000001, 0b11000000}, //'7'
      {0b00011111, 0b11000000}, //'8'
      {0b00011011, 0b11000000}, //'9'
      {0b00000000, 0b00000000}, //' '
  };
  void setTransferDigitPattern(unsigned int pattern);
  void setTransferBitPatternLed(unsigned int pattern, unsigned int &transferBitshift);
  void setTransferBitPatternControl(unsigned int pattern, unsigned int &transferBitshift);

public:
  DigitPattern(unsigned int row, unsigned int col);
  virtual ~DigitPattern(void);
  void setDigit(unsigned int row, unsigned int col, unsigned char digit);
  void setBitPattern(unsigned int pattern);
  void transferBitPattern(void);
};
DigitPattern::DigitPattern(unsigned int row, unsigned int col)
{
  digitPatternRow = row;
  digitPatternCol = col;
  digitPatternArray = new unsigned int *[digitPatternCol];
  for (unsigned int idx = 0; idx < digitPatternCol; ++idx)
  {
    digitPatternArray[idx] = new unsigned int[digitPatternRow];
  }
  transferDigitPatternByte = digitPatternCol * (digitPatternRow / 2);
  transferDigitPatternArray = new unsigned int[transferDigitPatternByte];
  transferBitPatternBit = 20;                             //controlBitPattern
  transferBitPatternBit += transferDigitPatternByte * 10; //ledBitPattern
  transferBitPatternByte = (transferBitPatternBit / 8) + ((transferBitPatternBit % 8 == 0) ? 0 : 1);
  transferBitPatternArray = new unsigned char[transferBitPatternByte];
}
DigitPattern::~DigitPattern(void)
{
  for (unsigned int idx = 0; idx < digitPatternCol; ++idx)
  {
    delete[] digitPatternArray[idx];
    digitPatternArray[idx] = NULL;
  }
  delete[] digitPatternArray;
  digitPatternArray = NULL;
  delete[] transferDigitPatternArray;
  transferDigitPatternArray = NULL;
  delete[] transferBitPatternArray;
  transferBitPatternArray = NULL;
}
void DigitPattern::setDigit(unsigned int row, unsigned int col, unsigned char digit)
{
  switch (digit)
  {
  case '0':
  case '1':
  case '2':
  case '3':
  case '4':
  case '5':
  case '6':
  case '7':
  case '8':
  case '9':
    digitPatternArray[col][row] = (digit - '0');
    break;
  case ' ':
    digitPatternArray[col][row] = 10;
    break;
  }
}
void DigitPattern::setTransferDigitPattern(unsigned int pattern)
{
  unsigned int index = 0;
  for (unsigned int colIdx = 0/*digitPatternCol-1*/; colIdx < digitPatternCol; ++colIdx)
  {
    for (unsigned int rowIdx = 0/*digitPatternRow-1*/; rowIdx < digitPatternRow; ++rowIdx)
    {
      if (rowIdx % 2 == pattern)
      {
        transferDigitPatternArray[(index++) % transferDigitPatternByte] = digitPatternArray[colIdx][rowIdx];
      }
    }
  }
Serial.print("S>");
Serial.print(transferDigitPatternArray[0]);
Serial.print(",");
Serial.print(transferDigitPatternArray[1]);
Serial.print("\n");
}
void DigitPattern::setTransferBitPatternLed(unsigned int pattern, unsigned int &transferBitshift)
{
  unsigned int index = 0;
  unsigned char high = 0;
  unsigned char low = 0;
  unsigned int readBitshift = 0;
  pattern = pattern;

  memset(transferBitPatternArray, 0, transferBitPatternByte);
  while (index < transferDigitPatternByte)
  {
    high = ledBitPattern[transferDigitPatternArray[index]].high;
    low = ledBitPattern[transferDigitPatternArray[index]].low;
    readBitshift = (transferBitshift % 8);
    for (unsigned int readBitIdx = 0; readBitIdx < 8; ++readBitIdx)
    {
      if ((readBitIdx + readBitshift) < 8)
      {
        transferBitPatternArray[transferBitshift / 8] |= ((0b10000000 >> (transferBitshift % 8)) & (high >> readBitshift));
      }
      else
      {
        transferBitPatternArray[transferBitshift / 8] |= ((0b10000000 >> (transferBitshift % 8)) & (high << (8 - readBitshift)));
      }
      ++transferBitshift;
    }
    readBitshift = (transferBitshift % 8);
    for (unsigned int readBitIdx = 0; readBitIdx < 2; ++readBitIdx)
    {
      if ((readBitIdx + readBitshift) < 8)
      {
        transferBitPatternArray[transferBitshift / 8] |= ((0b10000000 >> (transferBitshift % 8)) & (low >> readBitshift));
      }
      else
      {
        transferBitPatternArray[transferBitshift / 8] |= ((0b10000000 >> (transferBitshift % 8)) & (low << (8 - readBitshift)));
      }
      ++transferBitshift;
    }
    ++index;
  }
}
void DigitPattern::setTransferBitPatternControl(unsigned int pattern, unsigned int &transferBitshift)
{
  transferBitPatternArray[transferBitshift / 8] |= (0b00001111 & controlBitPattern[pattern].high);
  transferBitshift += 4;
  transferBitPatternArray[transferBitshift / 8] |= (0b11111111 & controlBitPattern[pattern].mid);
  transferBitshift += 8;
  transferBitPatternArray[transferBitshift / 8] |= (0b11111111 & controlBitPattern[pattern].low);
  transferBitshift += 8;
}
void DigitPattern::setBitPattern(unsigned int pattern)
{
  unsigned int transferBitshift = 0;
  if (pattern > 1)
  {
    return;
  }
  setTransferDigitPattern(pattern);

  //ledBitPattern
  setTransferBitPatternLed(pattern, transferBitshift);

  //controlBitPattern
  setTransferBitPatternControl(pattern, transferBitshift);
}
void DigitPattern::transferBitPattern(void)
{
  digitalWrite(SS, LOW);
  for (unsigned int index = 0; index < transferBitPatternByte; ++index)
  {
    SPI.transfer(transferBitPatternArray[index]);
  }
  digitalWrite(SS, HIGH);
}
const unsigned char _digitPattern_[] = {
    //abcdefgp
    0b11111100, //'0'
    0b01100000, //'1'
    0b11011010, //'2'
    0b11110010, //'3'
    0b01100110, //'4'
    0b10110110, //'5'
    0b10111110, //'6'
    0b11100000, //'7'
    0b11111110, //'8'
    0b11110110, //'9'
    0b00000000, //' '
};
volatile char dispDigitPatternTbl[4] = {'8', '8', '8', '8'};
const unsigned int dispDigitPatternMax = SIZEOFARRAY(dispDigitPatternTbl);
volatile unsigned int dispDigitPatternIdx = 0;
const unsigned char cAllBitOn[3] = {
    0b11111111,
    0b11111100,
    0b11110110, //9
};

void initKeyPad(void)
{
  for (unsigned int lpcnt = 0; lpcnt < SIZEOFARRAY(readPortId); ++lpcnt)
  {
    pinMode(readPortId[lpcnt], INPUT_PULLUP);
  }
  for (unsigned int lpcnt2 = 0; lpcnt2 < SIZEOFARRAY(writePortId); ++lpcnt2)
  {
    pinMode(writePortId[lpcnt2], OUTPUT);
    digitalWrite(writePortId[lpcnt2], HIGH);
  }
}
void scanKeyPad(void)
{
  int val = 0;
  for (unsigned int lpcnt2 = 0; lpcnt2 < SIZEOFARRAY(writePortId); ++lpcnt2)
  {
    digitalWrite(writePortId[lpcnt2], LOW);
    for (unsigned int lpcnt = 0; lpcnt < SIZEOFARRAY(readPortId); ++lpcnt)
    {
      val = digitalRead(readPortId[lpcnt]);
      if (val == LOW)
      {
        if (button[lpcnt2][lpcnt] < keyPadReadTimes)
        {
          ++(button[lpcnt2][lpcnt]);
          if (button[lpcnt2][lpcnt] == keyPadReadTimes)
          {
            keyPadChange = 1;
            dispDigitPatternTbl[0] = dispDigitPatternTbl[1];
            dispDigitPatternTbl[1] = dispDigitPatternTbl[2];
            dispDigitPatternTbl[2] = dispDigitPatternTbl[3];
            dispDigitPatternTbl[3] = keyPadTable[lpcnt2][lpcnt];
            if ('\r' == keyPadTable[lpcnt2][lpcnt])
            {
              dispDigitPatternTbl[0] = ' ';
              dispDigitPatternTbl[1] = ' ';
              dispDigitPatternTbl[2] = ' ';
              dispDigitPatternTbl[3] = ' ';
            }
          }
        }
      }
      else
      {
        button[lpcnt2][lpcnt] = 0;
      }
    }
    digitalWrite(writePortId[lpcnt2], HIGH);
  }
}
void timerOneFire()
{
  scanKeyPad();
}

class TransferData;
class LightsNumbersWithLEDDigit;
class LightsNumbersWithLEDLine;
class LightsNumbersWithLEDBord;
DigitPattern *digitPatternData;
class TransferData
{
protected:
  unsigned int bitPatternWroteBitSize;
  unsigned char bitPatternByteSize = ((1*4*10)/8);
  unsigned char *bitPatternArray;

public:
  TransferData()
  {
    bitPatternWroteBitSize = 0U;
    bitPatternArray = new unsigned char[bitPatternByteSize];
    memset(bitPatternArray,0x00,bitPatternByteSize);
  }
  ~TransferData()
  {
    delete[] bitPatternArray;
    bitPatternArray = NULL;
  }
  void append(LightsNumbersWithLEDDigit &ledDigit);
  void clear(void){
    bitPatternWroteBitSize = 0U;
    memset(bitPatternArray,0x00,bitPatternByteSize);
  }
  void transfer(void){
    digitalWrite(SS, LOW);
    for (unsigned int index = 0; index < bitPatternByteSize; ++index)
    {
      SPI.transfer(bitPatternArray[index]);
    }
    digitalWrite(SS, HIGH);
  }
};
class LightsNumbersWithLEDDigit
{
protected:
  static const unsigned short ledBitPattern[];
  char digit;

public:
  void setDigit(char dig)
  {
    digit = dig;
  };
  unsigned int getBitSize(void)
  {
    return 10;
  };
  unsigned short bitPattern(void);
  void setData(TransferData &transferData){
    transferData.append(*this);
  }
};
class LightsNumbersWithLEDLine
{
  protected:
  LightsNumbersWithLEDDigit ledDigit[4];
  public:
  void setDigit(int row, char dig){
    ledDigit[row].setDigit(dig);
  }
  unsigned int getBitSize(void){
    return 4*ledDigit[0].getBitSize();
  }
  void setData(TransferData &transferData){
    ledDigit[3].setData(transferData);
    ledDigit[2].setData(transferData);
    ledDigit[1].setData(transferData);
    ledDigit[0].setData(transferData);
  }
};
class LightsNumbersWithLEDBord
{
  protected:
  TransferData transferData;
  LightsNumbersWithLEDLine ledLine[1];
  public:
  LightsNumbersWithLEDBord(){
  }
  ~LightsNumbersWithLEDBord(){
  }
  void setDigit(int row, int col, char dig){
    ledLine[col].setDigit(row,dig);
  }
  void transfer(void){
    transferData.clear();
    ledLine[0].setData(transferData);
    transferData.transfer();
  }
};

void TransferData::append(LightsNumbersWithLEDDigit &ledDigit)
{
  //Fill from the end of the array
  unsigned short bitPattern = ledDigit.bitPattern();
  unsigned char bitSize = ledDigit.getBitSize();
  unsigned char bitWrote = 0U;
  while(bitSize > 0){
    bitPatternArray[bitPatternByteSize-1-(bitPatternWroteBitSize/8)] |= (unsigned char)(bitPattern<<(bitPatternWroteBitSize%8));
    bitWrote = (8-(bitPatternWroteBitSize%8));
    if(bitSize < bitWrote){
      bitWrote = bitSize;
    }
    bitPattern >>= bitWrote;
    bitPatternWroteBitSize += bitWrote;
    bitSize -= bitWrote;
  }
Serial.print("T>");
Serial.print(bitPatternWroteBitSize);
Serial.print(" ");
Serial.print(bitPattern);
Serial.print(" ");
Serial.print(bitPatternArray[0]);
Serial.print(" ");
Serial.print(bitPatternArray[1]);
Serial.print(" ");
Serial.print(bitPatternArray[2]);
Serial.print(" ");
Serial.print(bitPatternArray[3]);
Serial.print(" ");
Serial.print(bitPatternArray[4]);
Serial.print("\n");
}
const unsigned short LightsNumbersWithLEDDigit::ledBitPattern[11] = {
    //------zyxgfedcba
    0b0000000000111111, //'0'
    0b0000000000000110, //'1'
    0b0000000001011011, //'2'
    0b0000000001001111, //'3'
    0b0000000001100110, //'4'
    0b0000000001101101, //'5'
    0b0000000001111101, //'6'
    0b0000000000000111, //'7'
    0b0000000001111111, //'8'
    0b0000000001101111, //'9'
    0b0000000000000000, //' '
    //------zyxgfedcba
};
unsigned short LightsNumbersWithLEDDigit::bitPattern(void)
{
  switch (digit)
  {
  case '0':
  case '1':
  case '2':
  case '3':
  case '4':
  case '5':
  case '6':
  case '7':
  case '8':
  case '9':
    return ledBitPattern[digit - '0'];
  case ' ':
    return ledBitPattern[10];
  default:
    return 0U;
  }
}

LightsNumbersWithLEDBord *lightsNumbersWithLEDBord;

void setup()
{
  Serial.begin(115200);
Serial.print(millis());
Serial.print(",");
  pinMode(toneSpkPin, OUTPUT);
  initKeyPad();
  Timer1.initialize(10 * 1000); //マイクロ秒単位で設定
  Timer1.attachInterrupt(timerOneFire);
#if defined(USE_SPI)
  pinMode(SS, OUTPUT);
  digitalWrite(SS, HIGH);
  SPI.begin();
  //  SPI.beginTransaction(SPISettings(9600, firstBit, SPI_MODE0));
  SPI.setBitOrder(firstBit);
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  SPI.setDataMode(SPI_MODE0);
#else  // defined (USE_SPI)
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
#endif // defined (USE_SPI)
  selectDigit = 0;
  timeMicros = 0;
  timeMillis = 0;
  digitPatternData = new DigitPattern(4, 1);
  lightsNumbersWithLEDBord = new LightsNumbersWithLEDBord();
Serial.print(millis());
Serial.print(",");
}

unsigned int bootSoundMode = 0;
unsigned long bootSoundTime = 0;
const unsigned char bootSoundPiTime = 70;
const unsigned char bootSoundPoTime = 70;
unsigned int bootSoundTone(unsigned long nowtime)
{
  switch (bootSoundMode)
  {
  case 0:
    tone(toneSpkPin, 1000);
    bootSoundTime = nowtime;
    bootSoundMode = 1;
    break;
  case 1:
    if (nowtime > bootSoundTime + bootSoundPiTime)
    {
      tone(toneSpkPin, 2000);
      bootSoundMode = 2;
    }
    break;
  case 2:
    if (nowtime > bootSoundTime + bootSoundPiTime + bootSoundPoTime)
    {
      noTone(toneSpkPin);
      bootSoundMode = 3;
    }
    break;
  case 3:
  default:
    break;
  }
  return bootSoundMode;
}

unsigned int dispTgl = 0;
void loop()
{
  bootSoundTone(millis());
//  unsigned long nowTimeMicros = micros();
  unsigned long nowTimeMillis = millis();
//  unsigned char serialcmd[3] = {0x00, 0x00, 0x00};
  if (timeMillis < nowTimeMillis)
  {
    ++dispTgl;
    dispTgl %= 2;
  }
  else{
    return;
  }
  timeMillis = nowTimeMillis;
  timeMillis += 10;
// Serial.print("K>");
// Serial.print(dispDigitPatternTbl[0]);
// Serial.print(" ");
// Serial.print(dispDigitPatternTbl[1]);
// Serial.print(" ");
// Serial.print(dispDigitPatternTbl[2]);
// Serial.print(" ");
// Serial.print(dispDigitPatternTbl[3]);
// Serial.print("\n");
  lightsNumbersWithLEDBord->setDigit(0,0,dispDigitPatternTbl[0]);
  lightsNumbersWithLEDBord->setDigit(1,0,dispDigitPatternTbl[1]);
  lightsNumbersWithLEDBord->setDigit(2,0,dispDigitPatternTbl[2]);
  lightsNumbersWithLEDBord->setDigit(3,0,dispDigitPatternTbl[3]);
  lightsNumbersWithLEDBord->transfer();
  // digitPatternData->setDigit(0,0,dispDigitPatternTbl[0]);
  // digitPatternData->setDigit(1,0,dispDigitPatternTbl[1]);
  // digitPatternData->setDigit(2,0,dispDigitPatternTbl[2]);
  // digitPatternData->setDigit(3,0,dispDigitPatternTbl[3]);
  // digitPatternData->setBitPattern(dispTgl);
  // digitPatternData->transferBitPattern();
  return;
  #if 0
  if (dispTgl == 0)
  {
    serialcmd[0] |= (0b00001000);
    serialcmd[0] |= (0b11000000 & (_digitPattern_[dispDigitPatternTbl[0]] << 6));
    serialcmd[1] |= (0b00111111 & (_digitPattern_[dispDigitPatternTbl[0]] >> 2));
    serialcmd[2] |= (0b11111111 & (_digitPattern_[dispDigitPatternTbl[2]] << 0));
  }
  else
  {
    serialcmd[0] |= (0b00000100);
    serialcmd[0] |= (0b11000000 & (_digitPattern_[dispDigitPatternTbl[1]] << 6));
    serialcmd[1] |= (0b00111111 & (_digitPattern_[dispDigitPatternTbl[1]] >> 2));
    serialcmd[2] |= (0b11111111 & (_digitPattern_[dispDigitPatternTbl[3]] << 0));
  }
#if defined(USE_SPI)
  digitalWrite(SS, LOW);
  SPI.transfer(serialcmd[0]);
  SPI.transfer(serialcmd[1]);
  SPI.transfer(serialcmd[2]);
  digitalWrite(SS, HIGH);
#else  // defined (USE_SPI)
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, firstBit, serialcmd[0]);
  shiftOut(dataPin, clockPin, firstBit, serialcmd[1]);
  shiftOut(dataPin, clockPin, firstBit, serialcmd[2]);
  digitalWrite(latchPin, HIGH);
#endif // defined (USE_SPI)
  timeMicros = nowTimeMicros;
  timeMillis = nowTimeMillis;
#endif
}
