/***************************************************
  This is a library for the MCP23008 i2c port expander

  These displays use I2C to communicate, 2 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include <SoftWire.h>
#ifdef __AVR
  #include <avr/pgmspace.h>
#elif defined(ESP8266)
  #include <pgmspace.h>
#endif
#include "Adafruit_MCP23008.h"


////////////////////////////////////////////////////////////////////////////////
// RTC_DS1307 implementation

void Adafruit_MCP23008::begin(uint8_t addr, SoftWire* inSw) {
  if (addr > 7) {
    addr = 7;
  }
  i2caddr = addr;
  sw = inSw;

  // sw->begin();

  // set defaults!
  sw->beginTransmission(MCP23008_ADDRESS | i2caddr);
#if ARDUINO >= 100
  sw->write((byte)MCP23008_IODIR);
  sw->write((byte)0xFF);  // all inputs
  sw->write((byte)0x00);
  sw->write((byte)0x00);
  sw->write((byte)0x00);
  sw->write((byte)0x00);
  sw->write((byte)0x00);
  sw->write((byte)0x00);
  sw->write((byte)0x00);
  sw->write((byte)0x00);
  sw->write((byte)0x00);
#else
  sw->send(MCP23008_IODIR);
  sw->send(0xFF);  // all inputs
  sw->send(0x00);
  sw->send(0x00);
  sw->send(0x00);
  sw->send(0x00);
  sw->send(0x00);
  sw->send(0x00);
  sw->send(0x00);
  sw->send(0x00);
  sw->send(0x00);
#endif
  sw->endTransmission();

}

void Adafruit_MCP23008::begin(SoftWire* inSw) {
  begin(0, inSw);
}

void Adafruit_MCP23008::pinMode(uint8_t p, uint8_t d) {
  uint8_t iodir;


  // only 8 bits!
  if (p > 7)
    return;

  iodir = read8(MCP23008_IODIR);

  // set the pin and direction
  if (d == INPUT) {
    iodir |= 1 << p;
  } else {
    iodir &= ~(1 << p);
  }

  // write the new IODIR
  write8(MCP23008_IODIR, iodir);
}

uint8_t Adafruit_MCP23008::readGPIO(void) {
  // read the current GPIO input
  return read8(MCP23008_GPIO);
}

void Adafruit_MCP23008::writeGPIO(uint8_t gpio) {
  write8(MCP23008_GPIO, gpio);
}


void Adafruit_MCP23008::digitalWrite(uint8_t p, uint8_t d) {
  uint8_t gpio;

  // only 8 bits!
  if (p > 7)
    return;

  // read the current GPIO output latches
  gpio = readGPIO();

  // set the pin and direction
  if (d == HIGH) {
    gpio |= 1 << p;
  } else {
    gpio &= ~(1 << p);
  }

  // write the new GPIO
  writeGPIO(gpio);
}

void Adafruit_MCP23008::pullUp(uint8_t p, uint8_t d) {
  uint8_t gppu;

  // only 8 bits!
  if (p > 7)
    return;

  gppu = read8(MCP23008_GPPU);
  // set the pin and direction
  if (d == HIGH) {
    gppu |= 1 << p;
  } else {
    gppu &= ~(1 << p);
  }
  // write the new GPIO
  write8(MCP23008_GPPU, gppu);
}

uint8_t Adafruit_MCP23008::digitalRead(uint8_t p) {
  // only 8 bits!
  if (p > 7)
    return 0;

  // read the current GPIO
  return (readGPIO() >> p) & 0x1;
}

uint8_t Adafruit_MCP23008::read8(uint8_t addr) {
  sw->beginTransmission(MCP23008_ADDRESS | i2caddr);
#if ARDUINO >= 100
  sw->write((byte)addr);
#else
  sw->send(addr);
#endif
  sw->endTransmission();
  sw->requestFrom(MCP23008_ADDRESS | i2caddr, 1);

#if ARDUINO >= 100
  return sw->read();
#else
  return sw->receive();
#endif
}


void Adafruit_MCP23008::write8(uint8_t addr, uint8_t data) {
  sw->beginTransmission(MCP23008_ADDRESS | i2caddr);
#if ARDUINO >= 100
  sw->write((byte)addr);
  sw->write((byte)data);
#else
  sw->send(addr);
  sw->send(data);
#endif
  sw->endTransmission();
}
