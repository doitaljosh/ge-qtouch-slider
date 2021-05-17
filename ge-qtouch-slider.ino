/*
 * GE Appliances 191D7972 based on AT42QT2640 capacitive touch controller
 * 
 * SPI connector pinout:
 * J1 ________
 * __|        |__  1
 *|   ⃝  ⃝  ⃝  ⃝  ⃞   |    VREF/WS, DRDYN, MISO, SCLK, VCC
 *|   ⃝  ⃝  ⃝  ⃝  ⃝   |
 *|______________|    RSTN, CHANGEN, SSN, MOSI, GND
 *                 2
 * 
 * Arduino pins:
 * NDRDY: pin 8 (active low)
 * NSS: pin 9 (active high)
 * NRST: 10 (active low)
 * CHANGEN: 7 (active low)
 * MOSI: pin 11
 * MISO: pin 12
 * SCK: pin 13
 * 
 */


#include <SPI.h>

const int ssPin = 9; // Slave select pin
const int drdyPin = 8; // Data ready pin
const int rstPin = 10; // Reset pin
const int intPin = 7; // Interrupt pin

void setup() {
  Serial.begin(230400);
  SPI.begin();

  pinMode(drdyPin, INPUT);
  pinMode(ssPin, OUTPUT);
  pinMode(rstPin, OUTPUT);
  pinMode(intPin, INPUT);

  reset();  
}

void reset() {
  digitalWrite(rstPin, LOW);
  delay(100);
  digitalWrite(rstPin, HIGH);
  delay(100);
}

void commDelay() {
  delayMicroseconds(100);
  //while (digitalRead(drdyPin) == 1) {
    // Wait for data ready signal.
  //}
}

void initMsg() {
  Serial.println("Device initialized successfully");
  Serial.println("QTouch firmware version: ");
  Serial.print(transaction(1, 1, 0, 1)); // Read the firmware version
}

char* getSliderPos() {
  char values[5];
  int idx = 0;
  for (int i=0x0E; i<= 0x12; i++) {
    values[idx] = transaction(i, 1, 0, 1); // Iteratively read registers
    (idx == 4) ? (idx = 0) : (idx++);
    delay(0.1); // 100us processing delay
  }

  return values;
}

/*
 * @brief Read from or write to the touch controller's SPI interface.
 * @param reg Register to read from or write to. (must not exceed 11 bits).
 * @param len Number of registers to read or write to. (must not exceed 9 bits)
 * @param writeValue Value to write to a register. (Set to zero if reading)
 * @param ctx 0: write, 1: read
 */
unsigned int transaction(unsigned int reg, unsigned int len, unsigned int writeValue, unsigned int ctx) {

  unsigned int hdrByte0; // Byte 0 (low 8 bits of 11 bit address)
  struct {
    unsigned int ctx :1; // Read: 1, write: 0
    unsigned int nHigh :1; // Most significant bit of 9 bit address data length
    unsigned int unused :3 = 0x00; // 3 padding bits
    unsigned int addrHigh :3; // High 3 bits of 11 bit address
  } hdrByte1; // Byte 1 bitfield
  unsigned int hdrByte2; // Byte 2 (Low 8 bits of 9 bit address data length)

  writeValue = 0;
  byte byteIn = 0; // Incoming byte
  unsigned int dataOut = 0; // Outgoing buffer

  // The register value must not exceed 11 bits.
  if (reg > 0xFF) {
    
    hdrByte0 = reg & 0xFF;
    hdrByte1.addrHigh = reg & 0x700;
    
  } else {
    
    hdrByte0 = reg;
    hdrByte1.addrHigh = 0;
    
  }

  // The length value must not exceed 9 bits.
  if (len > 0xFF) {
    
    hdrByte2 = len & 0xFF;
    hdrByte1.nHigh = len & 0x100;
    
  } else {
    
    hdrByte2 = len;
    hdrByte1.nHigh = 0;
    
  }

  // Set the context bit.
  hdrByte1.ctx = ctx;

  // Serialize byte 1 from it's bitfield structure.
  unsigned int hdrByte1Ser; // Byte 1 serialized
  hdrByte1Ser = (hdrByte1.ctx << 7);
  hdrByte1Ser |= (hdrByte1.nHigh << 6);
  hdrByte1Ser |= (hdrByte1.unused << 3);
  hdrByte1Ser |= (hdrByte1.addrHigh);

  digitalWrite(ssPin, LOW); // Assert SS pin low to enable communication
  delayMicroseconds(5);

  // Process read/write contexts.
  switch (ctx) {
    
    case 0:
    
      // Send the header bytes, then the write value.
      SPI.transfer(hdrByte0);
      commDelay();
      SPI.transfer(hdrByte1Ser);
      commDelay();
      SPI.transfer(hdrByte2);
      commDelay();
      SPI.transfer(writeValue);
      commDelay();

      return 0;
      break;
      
    case 1:
    
      // Read a number (len) of bytes after sending header bytes.

      // First, send the header bytes.
      SPI.transfer(hdrByte0);
      commDelay();
      SPI.transfer(hdrByte1Ser);
      commDelay();
      SPI.transfer(hdrByte2);
      commDelay();

      // Shift out read bytes into buffer.
      if (len > 0) {
        dataOut = dataOut << 8; // Add a byte to the size of the output buffer.
        byteIn = SPI.transfer(0x00); // Send a null byte to request the next byte.
        dataOut = dataOut | byteIn; // Append the incoming byte to the buffer.
        commDelay();
        len--;
      }

      return dataOut; // Return the output buffer
      break;
      
    default:
    
      return -1;
      break;
  }
  
  digitalWrite(ssPin, HIGH); // Assert SS pin high to finish communication
  
}


void writeRegister(byte reg, byte value) {

  reg = reg << 2; // Convert uint8 to base 2

  byte data = reg | 0b00000000; // OR the data with the write command

  digitalWrite(ssPin, LOW); // Assert SS pin low to enable communication

  SPI.transfer(data); // Transfer the combined command and data fields
  commDelay();
  SPI.transfer(value); // Transfer the value to write to the register
  commDelay();

  digitalWrite(ssPin, HIGH); // Assert SS pin high to finish communication
  
}

/*
 * @brief Dumps all SPI registers to the console in a tabular format.
 * @param ctx 0 = 8 bit registers, 1 = 11 bit registers
 */
void regDump(int ctx) {
  
  unsigned int reg;
  char charbuf[2];
  int xVal = (ctx == 1) ? 0xFF : 0x0F;
  int yVal = (ctx == 1) ? 0x7F : 0x0F;
  
  Serial.print("  ");
  
  for (int i=0; i<=xVal; i++) {
    
    Serial.print(" ");
    sprintf(charbuf, "%02X", i);
    Serial.print(charbuf);
    
  }
  
  Serial.println();
  
  for (int i=0; i<=yVal; i++) {
    
    sprintf(charbuf, "%02X", i);
    Serial.print(charbuf);
    Serial.print(" ");
    
    for (int j=0; j<=xVal; j++)  {
      
      unsigned int reg;
      reg = (reg & 0xFF) | (i << 8);
      reg = (reg >> 8) | (j >> 8);

      sprintf(charbuf, "%02X", transaction(reg, 1, 0, 1));
      Serial.print(charbuf);
      Serial.print(" ");
      
    }
    
    Serial.println();
    
  }
}

void getValues() {

  char* values;
        
  if (digitalRead(drdyPin) == LOW) {  // If the DRDY pin is low, the controller is ready.
    values = getSliderPos(); // Store 8 bit values from all 5 slider wheels into an array
    
    Serial.print("Slider 1 value: ");
    Serial.print(transaction(0x0E, 1, 0, 1), HEX);
    Serial.println();
    Serial.print("Slider 2 value: ");
    Serial.print(transaction(0x0F, 1, 0, 1), HEX);
    Serial.println();
    Serial.print("Slider 3 value: ");
    Serial.print(transaction(0x10, 1, 0, 1), HEX);
    Serial.println();
    Serial.print("Slider 4 value: ");
    Serial.print(transaction(0x11, 1, 0, 1), HEX);
    Serial.println();
    Serial.print("Slider 5 value: ");
    Serial.print(transaction(0x12, 1, 0, 1), HEX);
    Serial.println();
        
  } else {
    Serial.println("Device is busy");
  }
}

void loop() {

  unsigned int chipId = transaction(0, 1, 0, 1);

  if (chipId == 0x1A) {
    
    initMsg(); // If the device ID register is probed successfully, print a success.

    attachInterrupt(intPin, getValues, FALLING); // Retrieve values whenever the controller signals a change.
    
    while (1) {
      
      getValues();
      
    }
    
  } else {
    // In the event that the ID register returns the wrong value, print a failure.
    Serial.println("Device initialization failed. Trying again...");
    Serial.print("Register dump:");
    Serial.println();
    regDump(1); // 16 bit register dump
    reset(); // Reset the controller.
    
  }

  getValues();
  
  delay(1000); // Retry probing the controller every second

}
