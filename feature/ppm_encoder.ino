/*
   PPM generator originally written by David Hasko
   on https://code.google.com/p/generate-ppm-signal/
*/

//////////////////////CONFIGURATION///////////////////////////////
#define CHANNEL_NUMBER 8  //set the number of chanels
#define CHANNEL_DEFAULT_VALUE 1500  //set the default servo value
#define FRAME_LENGTH 22500  //set the PPM frame length in microseconds (1ms = 1000�s)
#define PULSE_LENGTH 300  //set the pulse length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 6  //set PPM signal output pin on the arduino

/*this array holds the servo values for the ppm signal
  change theese values in your code (usually servo values move between 1000 and 2000)*/
int ppm[CHANNEL_NUMBER];
long lastPpm;

void ppm_setup() {

  //initiallize default ppm values
  for (int i = 0; i < CHANNEL_NUMBER; i++) {
    ppm[i] = CHANNEL_DEFAULT_VALUE;
  }

  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)

  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;

  TCCR1A |= (1 << COM1A1);    // set COM1A1 and clear COM1A0 in order to
  TCCR1A &= ~(1 << COM1A0);   //    clear OC1A (PA6) on next compare match

  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM13);
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();

}

void update_ppm() {

  ppm[0] = map(aileron, 0, 255, 1000, 2000);
  ppm[1] = map(elevator, 0, 255, 1000, 2000);
  ppm[2] = map(throttle, 0, 255, 1000, 2000);
  ppm[3] = map(rudder, 0, 255, 1000, 2000);

  ppm[4] = drone_settings & SETTINGS_HOME ? 2000 : drone_settings & SETTINGS_LEDS ? 1300 : 1000;
  //ppm[5] = drone_settings & SETTINGS_HOME ? 2000 : 1000;
  ppm[5] = map(kmob2, 0, 255, 1000, 2000);
  ppm[6] = drone_settings & SETTINGS_RTH ? 2000 : 1000;
  ppm[7] = drone_settings & SETTINGS_GPS ? 2000 : 1000;

  //ppm[8] = map(kmob1, 0, 255, 1000, 2000);
  //ppm[9] = map(kmob2, 0, 255, 1000, 2000);
  lastPpm = millis();
}

ISR(TIMER1_COMPA_vect) { //leave this alone
  static boolean state = true;

  TCNT1 = 0;

  if (millis() - lastPpm > 2000)
  {
    ppm[2] = 900; //Thortle failsafe
  }

  if (state) {  //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PULSE_LENGTH;
    state = false;
  } else { //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;

    digitalWrite(sigPin, !onState);
    state = true;

    if (cur_chan_numb >= CHANNEL_NUMBER) {
      cur_chan_numb = 0;
      calc_rest = calc_rest + PULSE_LENGTH;//
      OCR1A = (FRAME_LENGTH - calc_rest);
      calc_rest = 0;
    }
    else {
      OCR1A = (ppm[cur_chan_numb] - PULSE_LENGTH);
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }
  }
}
