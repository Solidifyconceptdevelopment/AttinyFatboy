#define PWM_PIN   5       //pin 8 on mystery µC
#define ON_PIN    10      //pin 2 on mystery µC
#define BTN_PIN   1       //pin 12 on mystery µC
#define CHRG_PIN  2       //pin 11 on mystery µC
#define LED_PIN   3       //pin 10 on mystery µC
#define USB_PIN   9       //pin 3 on mystery µC
#define DWN_PIN   8       //pin 5 on mystery µC

#define START_VALUE  25   //brigntness level at startup min 60 - max 880
#define B_STEPS      144  //number of brightness steps between min brightness and max brightness

#define LONGPRESS 800     //duration that indicates a long press in ms
#define DIM_DELAY 8       //pause between next dimming step in ms

#define LED_DELAY 200     //pause between blinking of the charging LED in ms

#define V_DELAY 30000     //pause between blinking at low voltage in ms

#define V_TRESHOLD 3050   //min voltage level before the lamp turns off in mV
#define V_BLINK    3150   //min voltage level before the lamp start blinking in mV

const uint16_t PROGMEM brightness[B_STEPS] = {    //brightness look-up table for more linear looking dimmings
  60,  61,  62,  64,  65,  66,  67,  69,  70,  72,  73,  74,  76,  77,  79,  81,
  82,  84,  85,  87,  89,  91,  92,  94,  96,  98, 100, 102, 104, 106, 108, 110,
  112, 114, 116, 118, 121, 123, 125, 128, 130, 132, 135, 137, 140, 143, 145, 148,
  151, 153, 156, 159, 162, 165, 168, 171, 174, 178, 181, 184, 187, 191, 194, 198,
  201, 205, 209, 212, 216, 220, 224, 228, 232, 236, 240, 245, 249, 253, 258, 262,
  267, 272, 276, 281, 286, 291, 296, 301, 306, 312, 317, 322, 328, 334, 339, 345,
  351, 357, 363, 369, 375, 382, 388, 395, 401, 408, 415, 422, 429, 436, 443, 451,
  458, 466, 474, 481, 489, 497, 505, 514, 522, 531, 539, 548, 557, 566, 575, 585,
  594, 604, 613, 623, 633, 643, 654, 664, 675, 697, 719, 742, 765, 790, 827, 880
};

boolean prev_btn = false;
boolean btn = false;
boolean bootup = true;
boolean usb = false;
boolean charged = false;

boolean LED_blink = false;

unsigned long start_TS, dim_TS, LED_TS, V_TS;

int b = START_VALUE;
int step_b = 1;

void setup() {
  //set up timer1 for 10-bit resolution, fast PWM,no prescaling
  TCCR1A = _BV(COM1B1) | _BV(WGM10) | _BV(WGM11);
  TCCR1B = _BV(CS10) | _BV(WGM12);              

  //defining input/output pins
  pinMode(PWM_PIN, OUTPUT);
  pinMode(ON_PIN, OUTPUT);
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(CHRG_PIN, INPUT);
  pinMode(USB_PIN, INPUT);
  pinMode(DWN_PIN, OUTPUT);

  //check if USB is connected
  usb = digitalRead(USB_PIN);

  //if no usb is connected, turn on the lamp
  if (!usb) {
    analogWrite(PWM_PIN, pgm_read_word(&brightness[START_VALUE]));

    // if the battery voltage is within allowable range, keep the lamp on
    long voltage = readVcc();
    if (voltage > V_TRESHOLD) {
      digitalWrite(ON_PIN, HIGH);
      digitalWrite(LED_PIN, HIGH);
      digitalWrite(DWN_PIN, HIGH);
    }
  }

  start_TS = millis();
}

void loop() {
  //check for button press and USB connection
  btn = !digitalRead(BTN_PIN);
  usb = digitalRead(USB_PIN);

  
  if (usb) {      //handle charging LED and turn off the lamp when USB is connected
    digitalWrite(PWM_PIN, LOW);
    digitalWrite(DWN_PIN, LOW);

    charged = digitalRead(CHRG_PIN);
    if (charged) {
      digitalWrite(LED_PIN, LOW);
    } else {
      if (millis() - LED_TS > LED_DELAY) {
        LED_TS = millis();
        LED_blink = !LED_blink;
        digitalWrite(LED_PIN, LED_blink);
      }
    }


  } else {   //if no USB is connected... 

    //check battery voltage every V_DELAY milliseconds
    if (millis() - V_TS > V_DELAY) {
      long voltage = readVcc();
      if (voltage < V_TRESHOLD) {
        digitalWrite(ON_PIN, LOW);    //turn everythin off when battery is too low
      } else if (voltage < V_BLINK) {
        digitalWrite(PWM_PIN, LOW);   //quickly blink the lamp when the voltage is getting critical
        delay(100);
        analogWrite(PWM_PIN, pgm_read_word(&brightness[b]));
      }
      V_TS = millis();
    }
    
    if (btn && !prev_btn ) {
      start_TS = millis();  //set the timestamp to check for long presses
    }

    if (btn && millis() - start_TS > LONGPRESS) { //if it was a long button press...
      if (millis() - dim_TS > DIM_DELAY) {        //and the dimming delay has expired
        dim_TS = millis();

        b += step_b;                          //increase/decrese brightness

        if (b > B_STEPS - 1) b = B_STEPS - 1; //cap brightness at maximum
        if (b < 0) b = 0;                     //cap brightness at minimum

        analogWrite(PWM_PIN, pgm_read_word(&brightness[b]));
      }
    }

    if (!btn && prev_btn) {
      step_b = -step_b; //reverse brightness dimming direction
      if (bootup) {
        bootup = false; //don't turn off the lamp when it has just been turned on 
      } else {
        if (millis() - start_TS < LONGPRESS) { //if it was a short button press, turn of the lamp
          digitalWrite(ON_PIN, LOW);
          digitalWrite(PWM_PIN, LOW);
        }
      }
    }
  }
  prev_btn = btn;
}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}
