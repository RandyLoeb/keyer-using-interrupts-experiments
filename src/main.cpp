#include <Arduino.h>
#include <queue>
#include <string>
#include <M5Stack.h>
//These define's must be placed at the beginning before #include "ESP32TimerInterrupt.h"
#define TIMER_INTERRUPT_DEBUG 1
#include "ESP32TimerInterrupt.h"
#include "ESP32_ISR_Timer.h"
volatile uint32_t lastMillis = 0;
// Init ESP32 timer 1
ESP32Timer ITimer(1);

// Init ESP32_ISR_Timer
ESP32_ISR_Timer ISR_Timer;

// this might be too low? 50 and 25 were too high for 20wpm
#define HW_TIMER_INTERVAL_MS 1

#define ditpin GPIO_NUM_2
#define dahpin GPIO_NUM_5

// queue to hold dits and dahs seen by the paddle monitorning,
// sound loop will pull from here
std::queue<String> ditsNdahQueue;

// sort of a a lockout
volatile bool soundPlaying = false;

// timer library seems to need this...
void IRAM_ATTR TimerHandler(void)
{
  ISR_Timer.run();
}

// timers kick off these two funcs below.
// has debugging to see if we are dead nuts accurate
void IRAM_ATTR doDits()
{
  static unsigned long previousMillis = lastMillis;
  unsigned long deltaMillis = millis() - previousMillis;
  ditsNdahQueue.push("dit");
#if (TIMER_INTERRUPT_DEBUG > 0)

  Serial.print("dit = ");
  Serial.println(deltaMillis);
#endif

  previousMillis = millis();
}

void IRAM_ATTR doDahs()
{
  static unsigned long previousMillis = lastMillis;
  unsigned long deltaMillis = millis() - previousMillis;
  ditsNdahQueue.push("dah");
#if (TIMER_INTERRUPT_DEBUG > 0)
  Serial.print("dah = ");
  Serial.println(deltaMillis);
#endif

  previousMillis = millis();
}

// holds our flags and timer handles
volatile bool detectInterrupts = false;
volatile bool ditPressed = false;
volatile bool dahPressed = false;
volatile int ditTimer;
volatile int dahTimer;
volatile int debounceDitTimer;
volatile int debounceDahTimer;
volatile int toneSilenceTimer;
volatile int ditDahSpaceLockTimer;
volatile bool ditLocked = false;
volatile bool dahLocked = false;

// this is triggerd by hardware interrupt indirectly
void IRAM_ATTR detectPress(volatile bool *locker, volatile bool *pressed, int timer, int lockTimer, int pin, String message)
{

  // locker is our debouce variable, i.e. we'll ignore any changes
  // to the pin during hte bounce
  if (!*locker)
  {

    // wait for interrupts to be turned on
    if (detectInterrupts)
    {

      // what was previous date of pin?
      int pressedBefore = *pressed;

      // get the pin
      *pressed = !digitalRead(pin);

      // pressed?
      if (*pressed && !pressedBefore)
      {
        ditsNdahQueue.push(message);
#if (TIMER_INTERRUPT_DEBUG > 0)

        Serial.println(message + "pressed");
#endif

        //kickoff either the dit or dah timer passed in
        //so it will keep injecting into the queue
        ISR_Timer.restartTimer(timer);
        ISR_Timer.enable(timer);
      }
      // released?
      else if (!*pressed && pressedBefore)
      {
        // released so stop the timer
        ISR_Timer.disable(timer);
      }
    }

    // lock and kickoff the debouncer
    *locker = true;
    ISR_Timer.restartTimer(lockTimer);
    ISR_Timer.enable(lockTimer);
  }
}

// these two are triggered by hardware interrupts
void IRAM_ATTR detectDitPress()
{
  detectPress(&ditLocked, &ditPressed, ditTimer, debounceDitTimer, ditpin, "dit");
}

void IRAM_ATTR detectDahPress()
{
  detectPress(&dahLocked, &dahPressed, dahTimer, debounceDahTimer, dahpin, "dah");
}

// fired by the timer that unlocks the debouncer indirectly
void IRAM_ATTR unlockDebouncer(void (*detectCallback)(), volatile bool *flagToFalse)
{
  *flagToFalse = false;
  detectCallback();
}

// fired by timer for unlocker direclty
void IRAM_ATTR unlockDit()
{
  unlockDebouncer(detectDitPress, &ditLocked);
}

void IRAM_ATTR unlockDah()
{
  unlockDebouncer(detectDahPress, &dahLocked);
}

// fired by timer that ends a sidetone segment
void IRAM_ATTR silenceTone()
{
  M5.Speaker.mute();

  // kickoff the spacing timer between dits & dahs
  ISR_Timer.disable(toneSilenceTimer);
  ISR_Timer.restartTimer(ditDahSpaceLockTimer);
  ISR_Timer.enable(ditDahSpaceLockTimer);
}

// fired by timer holds sound during dits & dah spacing
void IRAM_ATTR releaseLockForDitDahSpace()
{

  soundPlaying = false;
  ISR_Timer.disable(ditDahSpaceLockTimer);
}

void setup()
{
  M5.begin();
  Serial.begin(115200);
  Serial.println("In setup()");
  // put your setup code here, to run once:
  pinMode(ditpin, INPUT_PULLUP);
  pinMode(dahpin, INPUT_PULLUP);

  //seems like pin can only have one interrupt attached
  attachInterrupt(digitalPinToInterrupt(ditpin), detectDitPress, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(GPIO_NUM_39), detectDitRelease, RISING);

  attachInterrupt(digitalPinToInterrupt(dahpin), detectDahPress, CHANGE);

  // Using ESP32  => 80 / 160 / 240MHz CPU clock ,
  // For 64-bit timer counter
  // For 16-bit timer prescaler up to 1024

  // Interval in microsecs
  if (ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_MS * 1000, TimerHandler))
  {
    lastMillis = millis();
    Serial.println("Starting  ITimer OK, millis() = " + String(lastMillis));
  }
  else
    Serial.println("Can't set ITimer correctly. Select another freq. or interval");

  // Just to demonstrate, don't use too many ISR Timers if not absolutely necessary

  // this timer monitors the dit paddle held down
  ditTimer = ISR_Timer.setInterval(121L, doDits);

  // this timer monitors the dah paddle held down
  dahTimer = ISR_Timer.setInterval(241L, doDahs);

  // debouncers, needs some tweaking
  debounceDitTimer = ISR_Timer.setInterval(1L, unlockDit);
  debounceDahTimer = ISR_Timer.setInterval(10L, unlockDah);

  // timer to silence tone (could be used to unkey transmitter)
  toneSilenceTimer = ISR_Timer.setInterval(60L, silenceTone);

  // timer to unlock the sidetone (could be transmitter key)
  ditDahSpaceLockTimer = ISR_Timer.setInterval(60L, releaseLockForDitDahSpace);

  // not sure if disabled by default by do it
  ISR_Timer.disable(ditTimer);
  ISR_Timer.disable(dahTimer);
  ISR_Timer.disable(debounceDitTimer);
  ISR_Timer.disable(debounceDahTimer);
  ISR_Timer.disable(toneSilenceTimer);
  ISR_Timer.disable(ditDahSpaceLockTimer);

  // turn on the speaker
  M5.Speaker.begin();
}

void loop()
{
  // wait until now to enable interrupts, things should be
  // initialized
  detectInterrupts = true;

  while (!ditsNdahQueue.empty())
  {
    //soundPlaying is a kind of "lock" to make sure we wait for
    //the last sound, plus spacing to be done
    if (!soundPlaying)
    {
      String ditOrDah = ditsNdahQueue.front();
      ditsNdahQueue.pop();

      // start playing tone
      // apparently m5 speaker just plays tone continuously, that's fine,
      // we have a timer to shut it off.
      M5.Speaker.tone(600);

      // lock us up
      soundPlaying = true;

      // figure out when the tone should stop
      // (and/or transmitter unkey)
      unsigned int interval = 60L;
      if (ditOrDah == "dah")
      {
        interval = 180L;
      }

      // set up and start the timer that will stop the tone
      // (and/or transmitter unkey)
      ISR_Timer.changeInterval(toneSilenceTimer, interval);
      ISR_Timer.restartTimer(toneSilenceTimer);
      ISR_Timer.enable(toneSilenceTimer);
    }
    else
    {
      //just wait until sound ends
    }
  }
}