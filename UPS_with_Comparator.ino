/*
 * Proximity Sensor using ATTiny412
 * chip: ATTiny 412
 * clock: 20MHz Internal
 * millis()/micros() Timer: Enabled(default timer)
 * startup Time: 0ms
 * BOD Voltage level:4.2V(20 MHz or less)
 * save EEPROM: EEPROM retained
 * printf(): Default
 * wire(Wire.h/I2C) library mode: Master or Slave
 * WDT timeout: Disabled
 * WDT window : No delay before windows opens
 */
#include <EEPROM.h>

#define PRGM_PIN A6
#define LED_PIN A3
#define SIGNAL_IN A7 //A1
#define PNP_OUT A2 //A6
#define NPN_OUT A1 //A7
#define POP_MODE 3
#define EEPROM_ADDR 0x00
#define CALIBRATION_ADDR 0x10
// DAC ref calculation 2500/256 = 9.765 | A3->163 * 9.756 =1590  | 8F->143 * 9.756 =1395

volatile byte Mode = 0 , modeDisplayed = 0, blinkNo = 0, Prog_Trigger = 0, ProgFlag = 0, CProgFlag = 0 ;
volatile byte Calibrated = 0 , DAC_HIGH = 0, DAC_LOW = 0;
volatile uint16_t adcVal, lowCount = 0, highCount = 0;
volatile boolean pgmFlag = false;

ISR(ADC0_RESRDY_vect)
{
  /* Clear flag by writing '1': */
  ADC0.INTFLAGS = ADC_RESRDY_bm;
  adcVal = ADC0.RES;

  if (adcVal <= 500)
  {
    lowCount++;
    highCount = 0;
  }
  else
  {
    lowCount = 0;
    highCount++;
  }

  if (highCount >= 1000 )
  {
    pgmFlag = true;
    highCount = 0;
  }

  if (lowCount >= 1000 )
  {
    pgmFlag = false;
    lowCount = 0;
  }
}

ISR(AC0_AC_vect)
{
  volatile byte AC0_OUTPUT;
  if (CProgFlag == 1)
  {
    Calibrated = 1;
    AC0.STATUS = AC_CMP_bm;
    return;
  }

  if (AC0.STATUS & AC_STATE_bm) {
    DAC0.DATA = DAC_LOW ; //0x8F ; //1.4 v  //0x9A;                              //VREF.CTRLA = VREF_DAC0REFSEL_1V5_gc;
    AC0_OUTPUT = HIGH;
  }
  else {
    DAC0.DATA = DAC_HIGH ; //0xA3 ; //1.6 v  //0xFF;                             //VREF.CTRLA = VREF_DAC0REFSEL_2V5_gc;
    AC0_OUTPUT = LOW;
  }
  if (ProgFlag == 0)//working mode
  {
    //flag for normal working mode
    switch (Mode)
    {
      case 1:
        {
          if (AC0_OUTPUT)  //(digitalRead(SIGNAL_IN) == HIGH)
          {
            digitalWrite(PNP_OUT, HIGH);
            digitalWrite(LED_PIN, HIGH);
          }
          else
          {
            digitalWrite(PNP_OUT, LOW);
            digitalWrite(LED_PIN, LOW);
          }
        } break;
      case 2:
        {
          if (AC0_OUTPUT) //(digitalRead(SIGNAL_IN) == HIGH)
          {
            digitalWrite(PNP_OUT, LOW);
            digitalWrite(LED_PIN, LOW);
          }
          else
          {
            digitalWrite(PNP_OUT, HIGH);
            digitalWrite(LED_PIN, HIGH);
          }
        } break;
      case 3:
        {
          if (AC0_OUTPUT) //(digitalRead(SIGNAL_IN) == HIGH)
          {
            digitalWrite(NPN_OUT, HIGH);
            digitalWrite(LED_PIN, HIGH);
          }
          else
          {
            digitalWrite(NPN_OUT, LOW);
            digitalWrite(LED_PIN, LOW);
          }
        } break;
      case 4:
        {
          if (AC0_OUTPUT) //(digitalRead(SIGNAL_IN) == HIGH)
          {
            digitalWrite(NPN_OUT, LOW);
            digitalWrite(LED_PIN, LOW);
          }
          else
          {
            digitalWrite(NPN_OUT, HIGH);
            digitalWrite(LED_PIN, HIGH);
          }
        } break;
    }
  }
  else {//Programming mode
    for (int i = 0; i < 3; i++)
      delayMicroseconds(50000);
    if (AC0_OUTPUT == LOW ) //(digitalRead(SIGNAL_IN) == LOW)
    {
      digitalWrite(LED_PIN, HIGH);
      for (int i = 0; i < 5; i++)
        delayMicroseconds(50000);
      digitalWrite(LED_PIN, LOW);
      modeDisplayed = 1;
      Mode++;
      if (Mode > 6)Mode = 6;
    }
  }
  /* The interrupt flag has to be cleared manually */
  AC0.STATUS = AC_CMP_bm;
}

void AC0_init (void)
{
  /* Negative input uses internal reference - voltage reference should be enabled */
  VREF.CTRLA = VREF_DAC0REFSEL_2V5_gc ;  /* changed for this version -> Voltage reference to 2.5V */
  VREF.CTRLB = VREF_DAC0REFEN_bm;        /* AC0 DACREF reference enable: enabled */

  /* DAC - Digital to Analog Converter */
  DAC0.CTRLA = DAC_ENABLE_bm;            /* DAC Enable bit mask. */
  DAC0.DATA = 0xA3; //128;

  /*Select proper inputs for comparator*/
  AC0.MUXCTRLA = AC_MUXPOS_PIN0_gc | AC_MUXNEG_DAC_gc ;  /* DAC output */

  AC0.CTRLA = AC_ENABLE_bm              /* Enable analog comparator */
              | AC_HYSMODE_50mV_gc      /* Enable hysteresis @50mV  */
              | AC_INTMODE_BOTHEDGE_gc ;  /* Any Edge */
  AC0.INTCTRL = AC_CMP_bm;              /* Analog Comparator 0 Interrupt enabled */
}

void ADC0_init(void)
{
  ADC0.CTRLC = ADC_PRESC_DIV128_gc /* CLK_PER divided by 128 */
               | ADC_REFSEL_VDDREF_gc; /* VDD */ //
  ADC0.CTRLA = ADC_ENABLE_bm /* ADC Enable: enabled */
               | ADC_RESSEL_10BIT_gc; /* 10-bit mode */
  /* Select ADC channel */
  ADC0.MUXPOS = ADC_MUXPOS_AIN6_gc;
  /*Eanble Interrupt*/
  ADC0.INTCTRL = ADC_RESRDY_bm ; //0x01;
  /* Enable FreeRun mode */
  ADC0.CTRLA |= ADC_FREERUN_bm;
}


void setup() {
  //  Serial.begin(57600);
  byte CalVal;
  delay(500);
  /* Positive Input - Disable digital input buffer */
  PORTA.PIN7CTRL = PORT_ISC_INPUT_DISABLE_gc; //For analog SIGNAL_IN
  /* Disable digital input buffer */
  PORTA.PIN6CTRL |= PORT_ISC_INPUT_DISABLE_gc;
  /* Disable pull-up resistor */
  PORTA.PIN6CTRL &= ~PORT_PULLUPEN_bm;

  pinMode(PNP_OUT, OUTPUT);
  pinMode(NPN_OUT, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(PNP_OUT, LOW);
  digitalWrite(NPN_OUT, LOW);
  AC0_init();
  ADC0_init();

  CalVal = EEPROM.read(CALIBRATION_ADDR);
  if ((CalVal == 0xFF) || (CalVal == 0x0))
  {
    Calibrated = 0;
    DAC0.DATA = 0x0;
  }
  else {
    Calibrated = 1;
    DAC_HIGH = CalVal;
    DAC_LOW = (byte)(DAC_HIGH * 0.88 );
  }

  Mode = EEPROM.read(EEPROM_ADDR);
  //  Serial.println(Mode);
  if ((Mode == 0xFF) || (Mode == 0x0))
  {
    EEPROM.write(EEPROM_ADDR, POP_MODE);
    Mode = EEPROM.read(EEPROM_ADDR);
    //    Serial.println(Mode);
  }
  if (digitalRead(PRGM_PIN) == HIGH)
    modeDisplayed = 1;
  sei();            /*Global interrupts enabled */
  /* Start ADC conversion */
  ADC0.COMMAND = ADC_STCONV_bm;
}

void Process_Calibration()
{
  byte DAC_data = 0, blinkCount = 5;

  while (!Calibrated) {
    DAC0.DATA = DAC_data;
    delay(10);
    DAC_data++;
    if ((DAC_data & 0x01) ^ 1)
      PORTA.OUTSET |= PIN3_bm;
    else
      PORTA.OUTCLR |= PIN3_bm;
  }
  PORTA.OUTCLR |= PIN3_bm;
  CProgFlag = 0;
  DAC_data--;
  EEPROM.write(CALIBRATION_ADDR, DAC_data);
  DAC_HIGH =  EEPROM.read(CALIBRATION_ADDR);
  DAC_LOW = (byte)(DAC_HIGH * 0.88 );

  while (blinkCount--) { ///  while(true){  */if you need continuous blink once calibrated/*
    PORTA.OUTSET |= PIN3_bm; //digitalWrite(LED_PIN, HIGH);
    delay(250);
    PORTA.OUTCLR |= PIN3_bm; //digitalWrite(LED_PIN, LOW);
    delay(750);
  }
}


void loop() {
  // put your main code here, to run repeatedly:
  if ((!Calibrated ) && pgmFlag )
  {
    CProgFlag = 1;
    Process_Calibration();
  }
  else if (Calibrated)
  {
    if (!pgmFlag) // Working mode : pgmFlag=false
    {
      ProgFlag = 0;
      if ( modeDisplayed)
      {
        delay(2000);
        modeDisplayed = 0;

        if ( Mode == 6)
        {
          EEPROM.write(CALIBRATION_ADDR, 0xFF);// while(true); //continuous while loop
          Calibrated = 0;
          EEPROM.write(EEPROM_ADDR, POP_MODE);
          while (true) // blink LED to show that its looping in calibration reset
          {
            PORTA.OUTSET |= PIN3_bm;
            delay(50);
            PORTA.OUTCLR |= PIN3_bm;
            delay(300);
          }
        }
        else if (Mode > 4)
          Mode = 4;

        EEPROM.write(EEPROM_ADDR, Mode);
        blinkNo = EEPROM.read(EEPROM_ADDR);

        //      Serial.println(blinkNo);
        while (blinkNo--) {
          PORTA.OUTSET |= PIN3_bm; //digitalWrite(LED_PIN, HIGH);
          delay(250);
          PORTA.OUTCLR |= PIN3_bm; //digitalWrite(LED_PIN, LOW);
          delay(750);
        }
        delay(5000);
        AC0.CTRLA = AC_ENABLE_bm | AC_INTMODE_BOTHEDGE_gc | AC_HYSMODE_50mV_gc ;
        AC0_AC_vect();
      }
      else if (Prog_Trigger == 0)
      {
        Mode = EEPROM.read(EEPROM_ADDR);
        DAC0.CTRLA = DAC_ENABLE_bm;            /* DAC Enable bit mask. */
        DAC0.DATA = DAC_HIGH ; //0xA3; //163;
        AC0.CTRLA = AC_ENABLE_bm  | AC_INTMODE_BOTHEDGE_gc | AC_HYSMODE_50mV_gc;
        AC0.MUXCTRLA = AC_MUXPOS_PIN0_gc | AC_MUXNEG_DAC_gc ;  /* DAC output */
        AC0_AC_vect();
      }

      Prog_Trigger = 1;
    }
    else // Program mode : pgmFlag=true
    {
      if (Prog_Trigger)
      {
        ProgFlag = 1;
        Mode = 0;
        DAC0.CTRLA = DAC_ENABLE_bm;            /* DAC Enable bit mask. */
        DAC0.DATA = 0xA3; //163;
        AC0.CTRLA = AC_ENABLE_bm | AC_INTMODE_NEGEDGE_gc | AC_HYSMODE_50mV_gc ;
        AC0.MUXCTRLA = AC_MUXPOS_PIN0_gc | AC_MUXNEG_DAC_gc ;  /* DAC output */
        AC0.INTCTRL = AC_CMP_bm;
        digitalWrite(PNP_OUT, LOW);
        digitalWrite(NPN_OUT, LOW);
        digitalWrite(LED_PIN, LOW);
        Prog_Trigger = 0;
      }

    }
  }
}
