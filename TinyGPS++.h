/*
TinyGPS++ - a small GPS library for Arduino providing universal NMEA parsing
Based on work by and "distanceBetween" and "courseTo" courtesy of Maarten Lamers.
Suggestion to add satellites, courseTo(), and cardinal() by Matt Monson.
Location precision improvements suggested by Wayne Holder.
Copyright (C) 2008-2013 Mikal Hart
All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef __TinyGPSPlus_h
#define __TinyGPSPlus_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <limits.h>

/********************----- Options -----********************/
//#define TINYGPSPLUS_OPTION_NO_CUSTOM_FIELDS
//#define TINYGPSPLUS_OPTION_NO_STATISTICS
/**************************************************/

#define _GPS_VERSION "0.92" // software version of this library
#define _GPS_MPH_PER_KNOT 1.15077945
#define _GPS_MPS_PER_KNOT 0.51444444
#define _GPS_KMPH_PER_KNOT 1.852
#define _GPS_MILES_PER_METER 0.00062137112
#define _GPS_KM_PER_METER 0.001
#define _GPS_FEET_PER_METER 3.2808399
#define _GPS_MAX_FIELD_SIZE 15

struct RawDegrees
{
   uint16_t deg;
   uint32_t billionths;
   bool negative;
public:
   RawDegrees() : deg(0), billionths(0), negative(false)
   {}
};

struct LatLong
{
  RawDegrees lat;
  RawDegrees lng;
};

template<typename T>
struct TinyGPSDatum
{
  friend class TinyGPSPlus;
public:
  uint32_t age() const       { return this->isValid() ? millis() - lastCommitTime : static_cast<uint32_t>(ULONG_MAX); }
  bool isValid() const    { return (flags & FLAG_VALID) != 0; }
  bool isUpdated() const  { return (flags & FLAG_UPDATED) != 0; }
  T value()        { flags &= (~FLAG_UPDATED); return val; }
  
  TinyGPSDatum() : flags(FLAG_DEFAULT), val(T())
  {}
protected:
  enum {FLAG_DEFAULT=0, FLAG_VALID=(1<<0), FLAG_UPDATED=(1<<1)};
  uint8_t flags;
  T val, newval;
  uint32_t lastCommitTime;
};

struct TinyGPSLocation : public TinyGPSDatum<LatLong>
{
   friend class TinyGPSPlus;
public:
   const RawDegrees &rawLat()     { return value().lat; }
   const RawDegrees &rawLng()     { return value().lng; }
   double lat();
   double lng();

   TinyGPSLocation()
   {}

private:
   void commit();
   void setLatitude(const char *term);
   void setLongitude(const char *term);
};

struct TinyGPSDate : public TinyGPSDatum<uint32_t>
{
   friend class TinyGPSPlus;
public:
   uint16_t year();
   uint8_t month();
   uint8_t day();

   TinyGPSDate()
   {}

private:
   void commit();
   void setDate(const char *term);
};

struct TinyGPSTime : public TinyGPSDatum<uint32_t>
{
   friend class TinyGPSPlus;
public:
   uint8_t hour();
   uint8_t minute();
   uint8_t second();
   uint8_t centisecond();

   TinyGPSTime()
   {}

private:
   void commit();
   void setTime(const char *term);
};

struct TinyGPSDecimal : public TinyGPSDatum<uint32_t>
{
   friend class TinyGPSPlus;
public:

   TinyGPSDecimal()
   {}

private:
   void commit();
   void set(const char *term);
};

struct TinyGPSInteger : public TinyGPSDatum<uint32_t>
{
   friend class TinyGPSPlus;
public:
   TinyGPSInteger()
   {}

private:
   void commit();
   void set(const char *term);
};

struct TinyGPSSpeed : public TinyGPSDecimal
{
   double knots()    { return value() / 100.0; }
   double mph()      { return _GPS_MPH_PER_KNOT * value() / 100.0; }
   double mps()      { return _GPS_MPS_PER_KNOT * value() / 100.0; }
   double kmph()     { return _GPS_KMPH_PER_KNOT * value() / 100.0; }
};

struct TinyGPSCourse : public TinyGPSDecimal
{
   double deg()      { return value() / 100.0; }
};

struct TinyGPSAltitude : public TinyGPSDecimal
{
   double meters()       { return value() / 100.0; }
   double miles()        { return _GPS_MILES_PER_METER * value() / 100.0; }
   double kilometers()   { return _GPS_KM_PER_METER * value() / 100.0; }
   double feet()         { return _GPS_FEET_PER_METER * value() / 100.0; }
};

class TinyGPSPlus;

#ifndef TINYGPS_OPTION_NO_CUSTOM_FIELDS
class TinyGPSCustom : public TinyGPSDatum<uint8_t>
{
public:
   TinyGPSCustom() {};
   TinyGPSCustom(TinyGPSPlus &gps, const char *sentenceName, int termNumber);
   void begin(TinyGPSPlus &gps, const char *_sentenceName, int _termNumber);

   const char *value()     { this->value(); return buffer; }

private:
   void commit();
   void set(const char *term);

   char stagingBuffer[_GPS_MAX_FIELD_SIZE + 1];
   char buffer[_GPS_MAX_FIELD_SIZE + 1];
   const char *sentenceName;
   int termNumber;
   friend class TinyGPSPlus;
   TinyGPSCustom *next;
};
#endif

class TinyGPSPlus
{
public:
  TinyGPSPlus();
  bool encode(char c); // process one character received from GPS
  TinyGPSPlus &operator << (char c) {encode(c); return *this;}

  TinyGPSLocation location;
  TinyGPSDate date;
  TinyGPSTime time;
  TinyGPSSpeed speed;
  TinyGPSCourse course;
  TinyGPSAltitude altitude;
  TinyGPSInteger satellites;
  TinyGPSDecimal hdop;

  static const char *libraryVersion() { return _GPS_VERSION; }

  static double distanceBetween(double lat1, double long1, double lat2, double long2);
  static double courseTo(double lat1, double long1, double lat2, double long2);
  static const char *cardinal(double course);

  static int32_t parseDecimal(const char *term);
  static void parseDegrees(const char *term, RawDegrees &deg);
  
#ifndef TINYGPS_OPTION_NO_STATISTICS
  uint32_t charsProcessed()   const { return encodedCharCount; }
  uint32_t sentencesWithFix() const { return sentencesWithFixCount; }
  uint32_t failedChecksum()   const { return failedChecksumCount; }
  uint32_t passedChecksum()   const { return passedChecksumCount; }
#endif

private:
  bool sentenceHasFix() const
  {
    return (flags & FLAG_SENTENCE_HAS_FIX)!=0;
  }
  void setSentenceHasFix(bool const i_value)
  {
    if(i_value) {
      flags |= FLAG_SENTENCE_HAS_FIX;
    } else {
      flags &= ~FLAG_IS_CHECKSUM_TERM;
    }
  }
  enum {FLAG_DEFAULT=0, FLAG_IS_CHECKSUM_TERM=(1<<0), FLAG_SENTENCE_HAS_FIX=(1<<1)};
  enum {GPS_SENTENCE_GPGGA, GPS_SENTENCE_GPRMC, GPS_SENTENCE_OTHER};

  // parsing state variables
  uint8_t parity;
  uint8_t flags;
  char term[_GPS_MAX_FIELD_SIZE];
  uint8_t curSentenceType;
  uint8_t curTermNumber;
  uint8_t curTermOffset;

#ifndef TINYGPS_OPTION_NO_CUSTOM_FIELDS
  // custom element support
  friend class TinyGPSCustom;
  TinyGPSCustom *customElts;
  TinyGPSCustom *customCandidates;
  void insertCustom(TinyGPSCustom *pElt, const char *sentenceName, int index);
#endif
  
#ifndef TINYGPS_OPTION_NO_STATISTICS
  // statistics
  uint32_t encodedCharCount;
  uint32_t sentencesWithFixCount;
  uint32_t failedChecksumCount;
  uint32_t passedChecksumCount;
#endif

  // internal utilities
  int fromHex(char a);
  bool endOfTermHandler();
};

#endif // def(__TinyGPSPlus_h)
