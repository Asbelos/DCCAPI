
const byte MAX_LOCOS=20;


class DCCApi {
  public:

  static void begin();
  static void loop();

  // Public DCC API functions
  static void setSpeed(int loco, byte speed, bool forward);
  static int  readCV(int cv);
  static bool writeCVByte(int cv, int bValue) ;
  static bool writeCVBit(int cv, int bNum, int bValue);
  static void writeCVByteMain(int cab, int cv, int bValue);
  static void writeCVBitMain(int cab, int cv, int bNum, int bValue);
  static void setFunction( int cab, int fByte, int eByte);
  static void setAccessory(int aAdd, int aNum, int activate) ;
  static bool writeTextPacket( byte *b, int nBytes);

private: 
  struct LOCO {
     int loco;
       byte speed;
       bool forward;
  };
  static bool verifyCV(int cv,byte bValue);
  static void setSpeed(int loco, byte speed, bool forward, bool isReminder);
  static void updateLocoReminder(int loco, byte tSpeed, bool forward);
  static byte nextLoco;
  static LOCO speedTable[MAX_LOCOS];
  
};
