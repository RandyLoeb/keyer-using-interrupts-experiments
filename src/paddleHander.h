#include <Arduino.h>
#include <queue>
#include <string>
//These define's must be placed at the beginning before #include "ESP32TimerInterrupt.h"
//#define TIMER_INTERRUPT_DEBUG 1
#include "ESP32TimerInterrupt.h"
#include "ESP32_ISR_Timer.h"
// this might be too low? 50 and 25 were too high for 20wpm
//#define HW_TIMER_INTERVAL_MS 1

#define ditpin GPIO_NUM_2
#define dahpin GPIO_NUM_5
class PaddleHandler
{

    volatile uint32_t lastMillis = 0;
    // Init ESP32 timer 1
    //ESP32Timer ITimer(1);

    // Init ESP32_ISR_Timer
    ESP32_ISR_Timer *_ISR_Timer;

    // holds our flags and timer handles

    volatile bool ditPressed = false;
    volatile bool dahPressed = false;
    volatile int ditTimer;
    volatile int dahTimer;
    volatile int debounceDitTimer;
    volatile int debounceDahTimer;
    /* volatile int toneSilenceTimer;
    volatile int ditDahSpaceLockTimer; */
    volatile bool ditLocked = false;
    volatile bool dahLocked = false;
    // queue to hold dits and dahs seen by the paddle monitorning,
    // sound loop will pull from here
    std::queue<String> *_ditsNdahQueue;

    // sort of a a lockout
    volatile bool soundPlaying = false;

    /* // timer library seems to need this...
    void IRAM_ATTR TimerHandler(void)
    {
        ISR_Timer.run();
    } */

    // timers kick off these two funcs below.
    // has debugging to see if we are dead nuts accurate
    void IRAM_ATTR doDits()
    {
        static unsigned long previousMillis = lastMillis;
        unsigned long deltaMillis = millis() - previousMillis;
        _ditsNdahQueue->push("dit");
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
        _ditsNdahQueue->push("dah");
#if (TIMER_INTERRUPT_DEBUG > 0)
        Serial.print("dah = ");
        Serial.println(deltaMillis);
#endif

        previousMillis = millis();
    }

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
                    _ditsNdahQueue->push(message);
#if (TIMER_INTERRUPT_DEBUG > 0)

                    Serial.println(message + "pressed");
#endif

                    //kickoff either the dit or dah timer passed in
                    //so it will keep injecting into the queue
                    _ISR_Timer->restartTimer(timer);
                    _ISR_Timer->enable(timer);
                }
                // released?
                else if (!*pressed && pressedBefore)
                {
                    // released so stop the timer
                    _ISR_Timer->disable(timer);
                }
            }

            // lock and kickoff the debouncer
            *locker = true;
            _ISR_Timer->restartTimer(lockTimer);
            _ISR_Timer->enable(lockTimer);
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
        unlockDebouncer(this->detectDitPress, &ditLocked);
    }

    void IRAM_ATTR unlockDah()
    {
        unlockDebouncer(this->detectDahPress, &dahLocked);
    }

    /* // fired by timer that ends a sidetone segment
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
    } */

public:
    volatile bool detectInterrupts = false;
    PaddleHandler(ESP32_ISR_Timer *iSR_Timer, std::queue<String> *ditsNdahQueue)
    {
        this->_ISR_Timer = iSR_Timer;
        this->_ditsNdahQueue = ditsNdahQueue;
    }
    void initialize()
    {
        // put your setup code here, to run once:
        pinMode(ditpin, INPUT_PULLUP);
        pinMode(dahpin, INPUT_PULLUP);

        //seems like pin can only have one interrupt attached
        attachInterrupt(digitalPinToInterrupt(ditpin), [this]() {this->detectDitPress();}, CHANGE);
        //attachInterrupt(digitalPinToInterrupt(GPIO_NUM_39), detectDitRelease, RISING);

        attachInterrupt(digitalPinToInterrupt(dahpin), [this](){this->detectDahPress();}, CHANGE);

        // Using ESP32  => 80 / 160 / 240MHz CPU clock ,
        // For 64-bit timer counter
        // For 16-bit timer prescaler up to 1024

        // Interval in microsecs
        /*  if (ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_MS * 1000, TimerHandler))
        {
            lastMillis = millis();
            Serial.println("Starting  ITimer OK, millis() = " + String(lastMillis));
        }
        else
            Serial.println("Can't set ITimer correctly. Select another freq. or interval");
 */
        // Just to demonstrate, don't use too many ISR Timers if not absolutely necessary

        // this timer monitors the dit paddle held down
        this->ditTimer = _ISR_Timer->setInterval(121L, (*timer_callback) [this](){this->doDits();});

        // this timer monitors the dah paddle held down
        this->dahTimer = _ISR_Timer->setInterval(241L, (*timer_callback)[this](){this->doDahs();});

        // debouncers, needs some tweaking
        this->debounceDitTimer = _ISR_Timer->setInterval(1L, (*timer_callback)[this](){this->unlockDit();});
        this->debounceDahTimer = _ISR_Timer->setInterval(10L, (*timer_callback)[this](){this->unlockDah();});

        /*  // timer to silence tone (could be used to unkey transmitter)
        toneSilenceTimer = ISR_Timer.setInterval(60L, silenceTone);

        // timer to unlock the sidetone (could be transmitter key)
        ditDahSpaceLockTimer = ISR_Timer.setInterval(60L, releaseLockForDitDahSpace);
 */
        // not sure if disabled by default by do it
        this->_ISR_Timer->disable(this->ditTimer);
        this->_ISR_Timer->disable(this->dahTimer);
        this->_ISR_Timer->disable(this->debounceDitTimer);
        this->_ISR_Timer->disable(this->debounceDahTimer);
        /* ISR_Timer.disable(toneSilenceTimer);
        ISR_Timer.disable(ditDahSpaceLockTimer); */
    }
};