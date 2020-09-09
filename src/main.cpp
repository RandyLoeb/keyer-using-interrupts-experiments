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

#define HW_TIMER_INTERVAL_MS 1
#define ditpin GPIO_NUM_2
#define dahpin GPIO_NUM_5

void IRAM_ATTR TimerHandler(void)
{
  /* static bool toggle  = false;
  static bool started = false;
  static int timeRun  = 0; */
  //Serial.println("TimerHandler");
  ISR_Timer.run();

  /* // Toggle LED every LED_TOGGLE_INTERVAL_MS = 5000ms = 5s
  if (++timeRun == (LED_TOGGLE_INTERVAL_MS / HW_TIMER_INTERVAL_MS) )
  {
    timeRun = 0;

    if (!started)
    {
      started = true;
      pinMode(LED_BUILTIN, OUTPUT);
    }

    //timer interrupt toggles pin LED_BUILTIN
    digitalWrite(LED_BUILTIN, toggle);
    toggle = !toggle;
  } */
}



// In ESP32, avoid doing something fancy in ISR, for example complex Serial.print with String() argument
// The pure simple Serial.prints here are just for demonstration and testing. Must be eliminate in working environment
// Or you can get this run-time error / crash : "Guru Meditation Error: Core 1 panic'ed (Cache disabled but cached memory region accessed)"
void IRAM_ATTR doDits()
{
  static unsigned long previousMillis = lastMillis;
  unsigned long deltaMillis = millis() - previousMillis;

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

#if (TIMER_INTERRUPT_DEBUG > 0)
  Serial.print("dah = ");
  Serial.println(deltaMillis);
#endif

  previousMillis = millis();
}

volatile bool detectInterrupts = false;
volatile bool ditPressed = false;
volatile bool dahPressed = false;
volatile int ditTimer;
volatile int dahTimer;
volatile int debounceDitTimer;
volatile int debounceDahTimer;
volatile bool ditLocked = false;
volatile bool dahLocked = false;

void IRAM_ATTR detectPress(volatile bool *locker, volatile bool *pressed, int timer, int lockTimer, int pin, String message)
{
  if (!*locker)
  {
    //ISR_Timer.disable(unlockDitTimer);
    if (detectInterrupts)
    {

      int pressedBefore = *pressed;
      *pressed = !digitalRead(pin);

      if (*pressed && !pressedBefore)
      {
        Serial.println(message);
        ISR_Timer.restartTimer(timer);
        ISR_Timer.enable(timer);
      }
      else if (!*pressed && pressedBefore)
      {

        //Serial.println("dit released");
        ISR_Timer.disable(timer);
        //Serial.println("dr");
      }
    }
    *locker = true;
    ISR_Timer.restartTimer(lockTimer);
    ISR_Timer.enable(lockTimer);
    //ISR_Timer.setTimer()
  }
}

void IRAM_ATTR detectDitPress()
{
  detectPress(&ditLocked, &ditPressed, ditTimer, debounceDitTimer, ditpin, "dit pressed");
}

void IRAM_ATTR detectDahPress()
{
  detectPress(&dahLocked, &dahPressed, dahTimer, debounceDahTimer, dahpin, "dah pressed");
}

void IRAM_ATTR unlockDebouncer(void (*detectCallback)(), volatile bool *flagToFalse)
{
  *flagToFalse = false;
  detectCallback();
}

void IRAM_ATTR unlockDit()
{
  unlockDebouncer(detectDitPress, &ditLocked);
}

void IRAM_ATTR unlockDah()
{
  unlockDebouncer(detectDahPress, &dahLocked);
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

  /* ISR_Timer.setInterval(2000L, doingSomething2s);
  ISR_Timer.setInterval(5000L, doingSomething5s);
  ISR_Timer.setInterval(11000L, doingSomething11s);
  ISR_Timer.setInterval(101000L, doingSomething101s); */
  ditTimer = ISR_Timer.setInterval(121L, doDits);
  dahTimer = ISR_Timer.setInterval(241L, doDahs);
  debounceDitTimer = ISR_Timer.setInterval(1L, unlockDit);
  debounceDahTimer = ISR_Timer.setInterval(10L, unlockDah);
  ISR_Timer.disable(ditTimer);
  ISR_Timer.disable(dahTimer);
  ISR_Timer.disable(debounceDitTimer);
  ISR_Timer.disable(debounceDahTimer);
  
}

void loop()
{
  // put your main code here, to run repeatedly:
  detectInterrupts = true;
  delay(5000);
}