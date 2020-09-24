
/*
      This code was quick and dirty, based on a PCM audio example in the
      arduino playground: http://playground.arduino.cc/Code/PCMAudio

      It's been heavely modified for use with RC to generate something that's
      a bit like an engine sound. I've started work on making the program
      readable, still some to do though.
*/

#include "settings.h"
#include "idle.h"
#define BASE_RATE idle_sampleRate

// Mode settings - These could easily be 4 jumpers connected to spare pins, checked at startup to determine mode
boolean managedThrottle = true;     // Managed mode looks after the digipot if fitted for volume, and adds some mass to the engine
//boolean potThrottle = true;         // A pot connected to A1, 0-1023 sets speed

// Stuff not to play with!
#define SPEAKER 3                               // This is kept as 3, original code had 11 as option, but this conflicts with SPI
volatile uint16_t currentSmpleRate = BASE_RATE; // Current playback rate, this is adjusted depending on engine RPM
boolean audioRunning = false;                   // Audio state, used so we can toggle the sound system
uint16_t curVolume = 0;                         // Current digi pot volume, used for fade in/out
volatile uint16_t curEngineSample;              // Index of current loaded sample
uint8_t  lastSample;                            // Last loaded sample
int16_t  currentThrottle = 0;                   // 0 - 1000, a top value of 1023 is acceptable

#define DEBUG 0
#define debug_println(...) \
            do { if (DEBUG) Serial.println(__VA_ARGS__); } while (0)
#define debug_print(...) \
            do { if (DEBUG) Serial.print(__VA_ARGS__); } while (0)

void setup()
{
  if (DEBUG)
    Serial.begin(115200);
  //SPCR |= _BV(SPIE); // turn on interrupts

  // MCP4131 digi pot
  pinMode(POT_CS, OUTPUT);
  pinMode(POT_SCK, OUTPUT);
  pinMode(POT_SDO, OUTPUT);
  digitalWrite(POT_CS, HIGH);
  digitalWrite(POT_SCK, HIGH);
  digitalWrite(POT_SDO, HIGH);

  if(managedThrottle) writePot(0);
  else writePot(DEFAULT_VOLUME);

  // setup complete, so start making sounds
  startPlayback();
}

void loop()
{
  doPotThrottle();

  if(managedThrottle) manageSpeed();
}

/* _____ _               _   _   _
  |_   _| |__  _ __ ___ | |_| |_| | ___  ___
    | | | '_ \| '__/ _ \| __| __| |/ _ \/ __|
    | | | | | | | | (_) | |_| |_| |  __/\__ \
    |_| |_| |_|_|  \___/ \__|\__|_|\___||___/ */
#define ACC_RATE 2
#define MAX_RPM (1023)
#define IDLE_RPM 0
void doPotThrottle(){
  bool thr = digitalRead(2)?true:false;
  static int cthr = IDLE_RPM;
  if (thr)
  {
    //accelerate
    cthr += ACC_RATE;
    if (cthr > MAX_RPM)
      cthr = MAX_RPM;
  }
  else
  {
    //
    cthr -= 3*ACC_RATE;
    if (cthr < IDLE_RPM)
      cthr = IDLE_RPM;
  }
  //
  if(managedThrottle){
    //currentThrottle = thr?1023:0;//map (cthr, 0, MAX_RPM, 0, 1023);
    currentThrottle = cthr;//map (cthr, 0, MAX_RPM, 0, 1023);
  } else {
    currentSmpleRate = F_CPU / (BASE_RATE + long(cthr * TOP_SPEED_MULTIPLIER));
  }
  debug_print (F("digipot read: "));
  debug_print (thr);
  debug_print (F(" gear: "));
  //debug_print (cgear);
  debug_print (F(" throttle: "));
  debug_println (cthr);
  //
  return;
  //we don't use the code below
  
  if(managedThrottle){
    currentThrottle = analogRead(POT_PIN);
  }
  else {
    currentSmpleRate = F_CPU / (BASE_RATE + long(analogRead(POT_PIN) * TOP_SPEED_MULTIPLIER));
  }

}

/* __  __                 ____  _                 _       _   _
  |  \/  | __ _ ___ ___  / ___|(_)_ __ ___  _   _| | __ _| |_(_) ___  _ __
  | |\/| |/ _` / __/ __| \___ \| | '_ ` _ \| | | | |/ _` | __| |/ _ \| '_ \
  | |  | | (_| \__ \__ \  ___) | | | | | | | |_| | | (_| | |_| | (_) | | | |
  |_|  |_|\__,_|___/___/ |____/|_|_| |_| |_|\__,_|_|\__,_|\__|_|\___/|_| |_| */

void manageSpeed(){

  static int16_t prevThrottle = 0xFFFF;
  static int16_t currentRpm = 0;
  const  int16_t maxRpm = 8184;
  const  int16_t minRpm = 0;

  static unsigned long throtMillis;
  static unsigned long volMillis;

  // Engine RPM
  if(millis() - throtMillis > 5) {
    throtMillis = millis();

    if(currentThrottle +12 > currentRpm){
      currentRpm += 6;
      if(currentRpm > maxRpm) currentRpm = maxRpm;
      prevThrottle = currentThrottle;

    }
    else if(currentThrottle -15 < currentRpm){
      currentRpm -= 12;
      if(currentRpm < minRpm) currentRpm = minRpm;
      prevThrottle = currentThrottle;
    }

    //if(currentRpm >> 2 < 255) spiReturnByte = currentRpm >> 2;
    //else spiReturnByte = 255;
    //if(currentRpm >> 2 < 0) spiReturnByte = 0;

    currentSmpleRate = F_CPU / (BASE_RATE + long(currentRpm * TOP_SPEED_MULTIPLIER) );
  }


  // Engine Volume
  if(millis() - volMillis > 50) {
    volMillis = millis();

    int vol = map(currentThrottle, 0, 1023, VOL_MIN, VOL_MAX);

    if(vol > curVolume) curVolume = vol;
    else {
      curVolume -= (curVolume/10);
      if(curVolume < VOL_MIN) curVolume = VOL_MIN;
    }

    int lastVolume = 0xFFFF;
    if(curVolume != lastVolume){
      lastVolume = curVolume;
      writePot(curVolume);
    }

  }

}

/* ____  _       _   ____       _
  |  _ \(_) __ _(_) |  _ \ ___ | |_
  | | | | |/ _` | | | |_) / _ \| __|
  | |_| | | (_| | | |  __/ (_) | |_
  |____/|_|\__, |_| |_|   \___/ \__|
           |___/                     */

void writePot(uint8_t data){
  // This function should get a value from 0 - 127
  // It would be trivial to convert this to work with
  // an I2C device.

  if(data > VOL_MAX) data = VOL_MAX; // cap it just in case

  digitalWrite(POT_CS, LOW);
  shiftOut(POT_SDO, POT_SCK, MSBFIRST, 0x00);
  shiftOut(POT_SDO, POT_SCK, MSBFIRST, data);
  digitalWrite(POT_CS, HIGH);

}

/* ____   ____ __  __   ____       _
  |  _ \ / ___|  \/  | / ___|  ___| |_ _   _ _ __
  | |_) | |   | |\/| | \___ \ / _ \ __| | | | '_ \
  |  __/| |___| |  | |  ___) |  __/ |_| |_| | |_) |
  |_|    \____|_|  |_| |____/ \___|\__|\__,_| .__/
                                            |_|    */
void startPlayback()
{
  pinMode(SPEAKER, OUTPUT);
  audioRunning = true;

  // Set up Timer 2 to do pulse width modulation on the speaker pin.
  ASSR &= ~(_BV(EXCLK) | _BV(AS2));                         // Use internal clock (datasheet p.160)

  TCCR2A |= _BV(WGM21) | _BV(WGM20);                        // Set fast PWM mode  (p.157)
  TCCR2B &= ~_BV(WGM22);

  TCCR2A = (TCCR2A | _BV(COM2B1)) & ~_BV(COM2B0);           // Do non-inverting PWM on pin OC2B (p.155)
  TCCR2A &= ~(_BV(COM2A1) | _BV(COM2A0));                   // On the Arduino this is pin 3.
  TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10); // No prescaler (p.158)

  OCR2B = pgm_read_byte(&idle_data[0]);                     // Set initial pulse width to the first sample.

  // Set up Timer 1 to send a sample every interrupt.
  cli();

  TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);             // Set CTC mode (Clear Timer on Compare Match) (p.133)
  TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));             // Have to set OCR1A *after*, otherwise it gets reset to 0!

  TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10); // No prescaler (p.134)

  OCR1A = F_CPU / BASE_RATE;                                // Set the compare register (OCR1A).
                                                            // OCR1A is a 16-bit register, so we have to do this with
                                                            // interrupts disabled to be safe.

  TIMSK1 |= _BV(OCIE1A);                                   // Enable interrupt when TCNT1 == OCR1A (p.136)

  lastSample = pgm_read_byte(&idle_data[idle_length-1]);
  curEngineSample = 0;
  sei();


  uint8_t target = map(currentThrottle, 0, 1023, VOL_MIN, VOL_MAX); // Fadein the volume pot
  for(uint8_t i = 0; i < target; i ++){
    curVolume = i;
    writePot(curVolume);
    delay(1);
  }
}

void stopPlayback()
{
  // Fadeout the volume pot
  for(uint8_t i = curVolume; i > 0; i--){
    curVolume = i;
    writePot(i);
    delay(1);
  }

  audioRunning = false;

  TIMSK1 &= ~_BV(OCIE1A); // Disable playback per-sample interrupt.
  TCCR1B &= ~_BV(CS10);   // Disable the per-sample timer completely.
  TCCR2B &= ~_BV(CS10);   // Disable the PWM timer.

  digitalWrite(SPEAKER, LOW);
}

/* ___       _                             _
  |_ _|_ __ | |_ ___ _ __ _ __ _   _ _ __ | |_ ___
   | || '_ \| __/ _ \ '__| '__| | | | '_ \| __/ __|
   | || | | | ||  __/ |  | |  | |_| | |_) | |_\__ \
  |___|_| |_|\__\___|_|  |_|   \__,_| .__/ \__|___/
                                    |_|            */

// This is the main playback interrupt, keep this nice and tight!!
ISR(TIMER1_COMPA_vect) {
  OCR1A = currentSmpleRate;

  if (curEngineSample >= idle_length) { // Loop the sample
    curEngineSample = 0;
  }

  OCR2B = pgm_read_byte(&idle_data[curEngineSample]);

  ++curEngineSample;

}
