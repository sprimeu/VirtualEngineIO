//================================================
// For more info, see wiring diagram.
// This fits an Arduino Mega2560.
//================================================

#pragma once


//
// === Event reporting string format, defined via the protocol. === //
//

//Performance, is the reason that I didn't use a 'printf' system for
//  defining 'ICON 0' ~ 'ICON 9'. I used fixed strings instead.
#define EVFMT_NEWLINE      "\r\n" //New Line style: 0x0D Carriage Return + 0x0A Line Feed 

#define EVFMT_Icon(x)      ("ICON " #x EVFMT_NEWLINE)
#define EVFMT_Icon_PR(x)   ("ICON " #x " %c" EVFMT_NEWLINE)
#define EVFMT_Accelerate   ("ACCL %c" EVFMT_NEWLINE)
#define EVFMT_Brake        ("BREK %c" EVFMT_NEWLINE)
#define EVFMT_Gauge(x)     ("GAUG " #x " %u" EVFMT_NEWLINE)
#define EVFMT_Ignite       ("IGNT" EVFMT_NEWLINE)
#define EVFMT_Hazard       ("HZRD" EVFMT_NEWLINE)
#define EVFMT_Gear         ("GEAR%s" EVFMT_NEWLINE)

#define EVFMT_Gear_Hint1   ("")
#define EVFMT_Gear_Hint2   (" L")
#define EVFMT_Gear_Hint3   (" S")

//
// === List of all inputs === //
//
enum
{
  //keys that stimulate one of the Instument Cluster lights
  HIT_LBLINK = 0,
  HIT_ENGINE,
  HIT_OIL,
  HIT_BRAKEABS,
  HIT_BATTERY,
  HIT_SEATBELT,
  HIT_DOOR,
  HIT_LIGHT,
  HIT_HANDBRAKE,
  HIT_RBLINK,

  //accelerate/brake pedals
  HIT_ACCEL,
  HIT_BRAKE,

  //potentiometers
  HIT_FUELVALUE,
  HIT_OILVALUE,
  HIT_TERMOVALUE,

  //misc. digital keys
  HIT_IGNITION,
  HIT_HAZARD_LIGHT,

  //NEW: ability to change the target of the potentiometer
  HIT_SWITCH_ANALOG,

  //NEW: gear button
  HIT_GEAR,

  //Total number
  HIT_COUNT,
};

static const HIT_DESCRIPTOR hit_descriptor[HIT_COUNT] =
{
  //Columns
  // Input Index (Analog/Digital),      Report Format,        Key Mode
  { 22, EVFMT_Icon(0),       HIT_MODE_PRESS, },
  { 24, EVFMT_Icon(1),       HIT_MODE_PRESS, },
  { 26, EVFMT_Icon(2),       HIT_MODE_PRESS, },
  { 28, EVFMT_Icon(3),       HIT_MODE_PRESS, },
  { 30, EVFMT_Icon(4),       HIT_MODE_PRESS, },
  { 32, EVFMT_Icon(5),       HIT_MODE_PRESS, },
  { 34, EVFMT_Icon(6),       HIT_MODE_PRESS, },
  { 36, EVFMT_Icon_PR(7),    HIT_MODE_PRESS_RELEASE, },
  { 38, EVFMT_Icon(8),       HIT_MODE_PRESS, },
  { 40, EVFMT_Icon(9),       HIT_MODE_PRESS, },

  { 10, EVFMT_Accelerate,    HIT_MODE_PRESS_RELEASE, },
  { 9,  EVFMT_Brake,         HIT_MODE_PRESS_RELEASE, },

  { 0 | HIT_INDEX_FAKED,  EVFMT_Gauge(F),      HIT_MODE_ANALOG, }, //NEW: The analog pins are set to fake value (see below)
  { 1 | HIT_INDEX_FAKED,  EVFMT_Gauge(O),      HIT_MODE_ANALOG, },
  { 2 | HIT_INDEX_FAKED,  EVFMT_Gauge(T),      HIT_MODE_ANALOG, },

  { 12, EVFMT_Ignite,        HIT_MODE_PRESS, },
  { 11, EVFMT_Hazard,        HIT_MODE_PRESS, },

  { 8,  "",                  HIT_MODE_PRESS_RELEASE, },

  { 52, EVFMT_Gear,          HIT_MODE_GEAR, },
};

//Fake analogs
#define FAKE_ANALOG_COUNT       3
#define FAKE_ANALOG_REAL_PIN    0  //A0

