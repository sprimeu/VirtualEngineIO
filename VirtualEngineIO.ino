//======================================================================//
// Hardware Emulator of System Signals for ALTEN Virtual Cockpit (AVC)
// This piece of code, implements the firmware of the hardware that
//   emulates the signals from a the AVC engine. Along with the "virtual
//   (mock) engine", the CAN messages from the ECU are simulated.
//
// Implemented on Arduino Mega (Atmega1280)
// See wiring diagram for details
//
// NOTES:
// - This program handles all inputs in a polling-fashion, using a
//   timer with precision of 1ms. All timing is handled with this accuracy.
// - 4 categories of input IO are defined:
//   * Digital keys which whenever the key is pressed, a message is sent.
//   * Digital keys which whenever the key is pressed or released, a message is sent.
//   * Analog voltages which are reported as soon as an update happens.
//   * Digital key which whenever the key is released, a message is sent and the
//     holding time is reported (gear button).
// - All analog inputs are handled using a single potentiometer (NEW!)
//   The reason is to use only one potentiometer. It is possible to switch
//   between the 3 inputs to be simulated.
//
// Programmed by A. Nakisai, ALTEN Sverige AB
// Last Edit: 2019/02/27
// Version 1.7
//======================================================================//

#include <stdint.h>
#include <stdio.h>

//======================================================================//
// Declarative Configuration
//======================================================================//

//
// === Description of keys (digital) and values (analog potentiometer) === //
//

//We need description of every key/value
//Including:
//  - The GPIO/Analog pin (The digital pins are Active-Low, Pull-Up)
//  - Format of reporting a change
//  - Mode of the input

#define HIT_CATEGORY_MASK         (0x0F00)
#define HIT_CATEGORY_DIGITAL      (0x0000)
#define HIT_CATEGORY_ANALOG       (0x0100)

#define HIT_FLAG_FIRST_UPDATE     (0x1000)

#define HIT_MODE_PRESS                 (0 | HIT_CATEGORY_DIGITAL)  //Report when clicked
#define HIT_MODE_PRESS_RELEASE         (1 | HIT_CATEGORY_DIGITAL)  //Report when pressed or when released
#define HIT_MODE_GEAR                  (2 | HIT_CATEGORY_DIGITAL)  //For the NEW gear button
#define HIT_MODE_ANALOG                (0 | HIT_CATEGORY_ANALOG)   //Report on any analog value change

#define HIT_INDEX_FAKED		           (0x100)
#define HIT_INDEX_FAKE_FLAG		       (0xFF)

#define ANALOG_MAX_A                   (1023)
#define ANALOG_MAX_B                   (100)

struct HIT_DESCRIPTOR
{
  int pin;                      //could be digital or analog pin, depending on
  const char* report_format;    //how to send an event (string format)
  uint16_t mode;                //mode
};

//List of all keys and analog potentiometers
#include "InputList.h"

//Input Signal Filtering
//For digital keys:
//  Amount of the time for key deboucing
#define HIT_FILTER_DEBOUNCE_TIME         ((uint8_t)20)           //Unit: ms

//For analog volumes:
//  Number of samples for averaging
#define HIT_FILTER_AVERAGING_COUNT       ((uint16_t)128)         //Unit: no unit, just a count

//NEW: We record the amount of time the key is pressed.
#define HIT_MAX_HOLD_TIME                10000
#define HIT_MAX_RELEASE_TIME             10000
#define HIT_GEAR_RELEASE_REPORT_TIME      250
#define HIT_GEAR_HOLD_REPORT_TIME         750

//Global filtering counter for analog pins
uint16_t Analog_Filter_Counter = 0;

//Status of each individual input
typedef uint32_t accum_t;
typedef uint16_t value_t;

struct HIT_STATE
{
  //last recorded state (for digital pins: 0/1, for analog pins: 0~100)
  value_t last_value;

  //analog accumulator for averaging
  accum_t accum;

  //any update occured? Should the value be reported?
  bool updated;

  //filtering counter for digital inputs
  uint8_t digital_filter_time;

  //hold down time for digital keys
  uint16_t hold_time;

  //release time for digital keys
  uint16_t release_time;

  //number of clicks (single, double, triple).
  int click_count;

  //ctor (to reset to default value)
  HIT_STATE() :
    last_value(),
    accum(),
    updated(),
    digital_filter_time(),
    hold_time(),
    release_time(),
    click_count()
  {
  }
};

HIT_STATE hit_state[HIT_COUNT];

//
// === List of other GPIO pins ===
//
#define PIN_LEN                    13

//Blinking LED
#define LED_BLINKTIME_TOTAL        500
#define LED_BLINKTIME_MID          480
uint16_t LED_Timer = 0;

//
// === We handle communications over serial (UART) protocol. ===
//
// Specify the instance of Serial
#define COMM                       (Serial)

// Specify the default baudrate (see the note in setup())
#define COMM_DEFAULT_BR            115200

//======================================================================//
// Function Declaration
//======================================================================//
void setup();
void loop();
void PollInputStates(uint8_t elapsed_time); //unit of elasped time: ms
void ReportInputStates();
void ClearChangeFlags();
void UpdateAnalogFaker();
uint16_t GetAnalogValue(uint16_t index);

//======================================================================//
// The Initialization Entry Point
//======================================================================//
void setup()
{
  //Setup the LED GPIO
  pinMode(PIN_LEN, OUTPUT);

  //Some blinking
  for (int i = 0; i < 5; i++)
  {
    digitalWrite(PIN_LEN, 0);
    delay(100);
    digitalWrite(PIN_LEN, 1);
    delay(100);
  }

  //Initiate the serial library.
  //If the Arduino has an internal USB CDC driver,
  //  the baudrate is irrelevant and is ignored.
  COMM.begin(COMM_DEFAULT_BR);

  //Set I/O states for each input.
  //Schedule an initial 'update' report depending on mode.
  for (int i = 0; i < HIT_COUNT; i++)
  {
    hit_state[i].updated =
      (hit_descriptor[i].mode & HIT_FLAG_FIRST_UPDATE) != 0;
    
    if ((hit_descriptor[i].mode & HIT_CATEGORY_MASK) == HIT_CATEGORY_DIGITAL)
    {
      pinMode( hit_descriptor[i].pin, INPUT_PULLUP );
    }
  }

  //Reset global signal filtering counter
  Analog_Filter_Counter = 0;
}

//======================================================================//
// The Initialization Entry Point
//======================================================================//
void loop()
{
  uint32_t last_time, cur_time;
  uint16_t elapsed_time;

  //We keep track of time, using a precise timer.
  //Every (at least) 1ms, we poll for state of inputs.
  last_time = (uint32_t)millis();
  while (1)
  {
    cur_time = (uint32_t)millis();
    if (cur_time < last_time)
    {
      //Timer rolled over
    }
    else if (cur_time > last_time)
    {
      //At least 1 millisecond passed
      //Let's assume that the elapsed time fits in 16-bit int.
      elapsed_time = (uint16_t)(cur_time - last_time);

      //Limit the elapsed time
      if (elapsed_time > 100)
      {
        elapsed_time = 100;
      }

      PollInputStates((uint8_t)elapsed_time);
    }
    last_time = cur_time;

    //Report any changes
    ReportInputStates();

    //Update fake analog input mechanism
    UpdateAnalogFaker();

    //Clear value update flags
    ClearChangeFlags();
  }
}

void ClearChangeFlags()
{
  HIT_STATE* state;
  int i;

  //Iterate over all inputs.
  //Clear all 'updated' flag.
  for (i = 0, state = &hit_state[0];
       i < HIT_COUNT;
       i++, state++)
  {
    if (state->updated)
    {
      state->updated = false;

      //Clear intermediate information
      state->click_count = 0;
    }
  }
}

//======================================================================//
// Report if state of any input is changed.
//======================================================================//
void ReportInputStates()
{
  char report[32];
  const char* exStr = nullptr;
  HIT_STATE* state;
  const HIT_DESCRIPTOR* desc;
  int i;

  //Iterate over all inputs.
  //Each input has an State and a Descriptor.
  for (i = 0, state = &hit_state[0], desc = &hit_descriptor[0];
       i < HIT_COUNT;
       i++, state++, desc++)
  {
    if (state->updated == false)
      //no recent change
      continue;

    //Report
    switch (desc->mode)
    {
    case HIT_MODE_PRESS:
      //Only report the transitions to '1'.
      if (state->last_value == 1)
      {
        COMM.write(desc->report_format);
      }
      else
      {
        //Do not report if value is 0.
      }
      break;
    case HIT_MODE_PRESS_RELEASE:
      //Report the digital value
      //Send over UART
      sprintf(report,
              desc->report_format,
              (state->last_value != 0 ? 'P' : 'R'));
      COMM.write(report);
      break;
    case HIT_MODE_GEAR:
      //If the key is released for more than click-delay time (see PollInputStates()), report.
      if (state->click_count == 1 && state->hold_time < HIT_GEAR_HOLD_REPORT_TIME)
      {
        //Click once, short holding time.
        exStr = EVFMT_Gear_Hint1;
      }
      else if (state->click_count == 1)
      {
        //Click once, long holding time
        exStr = EVFMT_Gear_Hint2;
      }
      else if (state->click_count == 2 && state->hold_time < HIT_GEAR_HOLD_REPORT_TIME)
      {
        //Double click, short holding time
        exStr = EVFMT_Gear_Hint3;
      }
      else
      {
        break;
      }
        
      sprintf(report,
              desc->report_format,
              exStr);
      COMM.write(report);
      break;
    case HIT_MODE_ANALOG:
      //Report the analog value
      //Send over UART
      sprintf(report,
              desc->report_format,
              state->last_value);
      COMM.write(report);
      break;
    }
  }
}

//======================================================================//
// NEW: Fake analog inputs:
//   As we only have one POTentiometer for the whole board,
//   we simulated having 3 actual POTs by a button which 'switches'
//   between which one gets the current value of the POT
//======================================================================//

uint16_t FakeAnalog_Selection = 0;
uint16_t FakeAnalog_Last[FAKE_ANALOG_COUNT];
uint16_t FakeAnalog_LastPot = 0;

void UpdateAnalogFaker()
{
  //Switch between analogs?
  if (hit_state[HIT_SWITCH_ANALOG].updated &&
      hit_state[HIT_SWITCH_ANALOG].last_value == 1)
  {
    //Key hit
    FakeAnalog_Selection++;
    if (FakeAnalog_Selection >= FAKE_ANALOG_COUNT)
    {
      FakeAnalog_Selection = 0;
    }
  }
}

uint16_t GetAnalogValue(uint16_t index)
{
  if (index <= HIT_INDEX_FAKE_FLAG)
  {
    //Read from analogRead.
    return analogRead(index);
  }
  else
  {
    //A fake analog input
    index = index & HIT_INDEX_FAKE_FLAG;
    if (index >= FAKE_ANALOG_COUNT)
    {
      //Out of range
      return 0;
    }

	//Read the documentation for more information.
	//There is a differential mechanism here.
    uint16_t Real = 0;
    int16_t V = 0;
    int16_t Diff = 0;
    if (FakeAnalog_Selection == index)
    {
      //If it is the selected input, manipulate using differential data from analogRead from the actual pin.
      Real = analogRead(FAKE_ANALOG_REAL_PIN);
      Diff = (int16_t)FakeAnalog_LastPot - (int16_t)Real;
      FakeAnalog_LastPot = Real;

      V = (int16_t)FakeAnalog_Last[index];
      V = V + Diff;
      if (V < 0)
        V = 0;
      else if (V > ANALOG_MAX_A)
        V = ANALOG_MAX_A;
    }
    else
    {
      V = (int16_t)FakeAnalog_Last[index];
    }

    //Save value.
    FakeAnalog_Last[index] = (uint16_t)V;

    //Return
    return (uint16_t)V;
  }
}

//======================================================================//
// Process state of inputs
//======================================================================//
void PollInputStates(uint8_t elapsed_time)
{
  HIT_STATE* state;
  const HIT_DESCRIPTOR* desc;
  int i;
  value_t value_analog;
  uint8_t value_digital;
  bool do_average = false;

  //Let the LED blink
  LED_Timer += elapsed_time;
  digitalWrite(PIN_LEN, LED_Timer > LED_BLINKTIME_MID);
  if (LED_Timer > LED_BLINKTIME_TOTAL)
  {
    LED_Timer -= LED_BLINKTIME_TOTAL;
  }

  //Update timer/counter of the global analog filter
  Analog_Filter_Counter++;
  if (Analog_Filter_Counter == HIT_FILTER_AVERAGING_COUNT)
  {
    do_average = true;
    Analog_Filter_Counter = 0;
  }

  //Iterate over all inputs.
  //Each input has an State and a Descriptor.
  for (i = 0, state = &hit_state[0], desc = &hit_descriptor[0];
       i < HIT_COUNT;
       i++, state++, desc++)
  {
    //Read the current value
    if ((desc->mode & HIT_CATEGORY_MASK) == HIT_CATEGORY_DIGITAL)
    {
      //A digital input
      bool toggled = false;

      //If the key is in 'pressed' state, update the holding timer.
      if (state->last_value != 0)
      {
        state->hold_time = state->hold_time + elapsed_time;
        if (state->hold_time > HIT_MAX_HOLD_TIME)
        {
          state->hold_time = HIT_MAX_HOLD_TIME;
        }
      }
      else
      {
        //Otherwise, update the release time.
        state->release_time = state->release_time + elapsed_time;
        if (state->release_time > HIT_MAX_RELEASE_TIME)
        {
          state->release_time = HIT_MAX_RELEASE_TIME;
        }
      }
      
      //Read its value
      //Note that digital inputs are active low.
      value_digital = (value_t)!digitalRead(desc->pin);

      //If there is a toggle, then run the filtering counter
      if ((uint8_t)state->last_value != value_digital)
      {
        //Wait for a short period.
        state->digital_filter_time += (uint8_t)elapsed_time;
        if (state->digital_filter_time >= HIT_FILTER_DEBOUNCE_TIME)
        {
          //Reached the debouncing time threshold. Register new value.
          state->digital_filter_time = 0;
          state->last_value = (value_t)value_digital;
          toggled = true;

          if (state->last_value == 1)
          {
            //Begin counting hold time
            state->hold_time = 0;
          }
          else
          {
            //Begin counting release time
            state->release_time = 0;
          }
        }
      }
      else
      {
        //No toggle occured since last registeration of value.
        state->digital_filter_time = 0;
      }

      //The higher level logic for the key.
      if (desc->mode != HIT_MODE_GEAR)
      {
        //No special high level logic
        if (toggled)
          state->updated = true;
      }
      else
      {
        //Upon transition to 1, increment number of sequentials clicks.
        if (toggled && state->last_value == 1)
        {
          state->click_count++;
        }

        //If the key is released for more than a threshold, report the number of clicks.
        if (state->last_value   == 0 &&
            state->click_count  != 0 &&
            state->release_time > HIT_GEAR_RELEASE_REPORT_TIME)
        {
          //Report
          state->updated = true;
        }

        //Also, if the key is held for more than a threshold, report.
        if (state->last_value   == 1 &&
            state->click_count  != 0 &&
            state->hold_time >   HIT_GEAR_HOLD_REPORT_TIME)
        {
          //Report
          state->updated = true;
        }
      }
    }
    else if ((desc->mode & HIT_CATEGORY_MASK) == HIT_CATEGORY_ANALOG)
    {
      //Analog input
      value_analog = (value_t)GetAnalogValue(desc->pin);

      //Update accumulator
      state->accum = state->accum + value_analog;

      //Do the averaging?
      if (do_average)
      {
        //Divide by the number of samples
        value_analog = state->accum / HIT_FILTER_AVERAGING_COUNT;
        state->accum = 0;

        //Convert it from range of 0~1023 to range of 0~100.
        value_analog = (uint16_t)((uint32_t)value_analog * ANALOG_MAX_B / ANALOG_MAX_A);

        //Value could not be negative.
        if (value_analog > ANALOG_MAX_B)
        {
          value_analog = ANALOG_MAX_B;
        }

        //Any change?
        if (value_analog != state->last_value)
        {
          //Change occured
          state->last_value = value_analog;
          state->updated = true;
        }
      }
    }

    //Done with this input :)
  }
}

//EOF
