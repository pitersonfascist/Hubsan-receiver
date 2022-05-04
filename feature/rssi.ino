
#define ADCchannel 0

#define conversion_not_done()  ADCSRA & (1<<ADSC)
#define conversion_done() !(ADCSRA & (1<<ADSC))
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
 
void init_rssi()
{
    // Select Vref=AVcc
    ADMUX |= (1 << REFS0);
    //set prescaller to 128 and enable ADC
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADEN);
    ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);
}

int read_adc_value()
{
    uint8_t low, high;
    low = ADCL;
    high = ADCH;
    // combine the two bytes
    return (high << 8) | low;
}

void update_rssi()
{
    if (conversion_done())
    {
        cbi(ADCSRA,ADIF);
        uint16_t val = read_adc_value();
        if (val < 150)
        {
            uint16_t oldrssi = rssi;
            //rssi = ((rssi << 8) - oldrssi) + ADC;
            rssi = rssi*0.99 + val*0.01 ;
            percentrssi = 100 * (130 - rssi) / 130.0;
        }
        ADCSRA |= (1<<ADSC);
    }
}
