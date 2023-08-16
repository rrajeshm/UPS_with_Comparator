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

#define PRGM_PIN A2
#define LED_PIN A3
#define SIGNAL_IN A7 //A1
#define PNP_OUT A6
#define NPN_OUT A1 //A7
#define POP_MODE 3
#define EEPROM_ADDR 0x10

volatile byte Mode = 0 , modeDisplayed = 0, blinkNo = 0, Prog_Trigger = 0, ProgFlag = 0 ;


ISR(AC0_AC_vect)
{
  volatile byte AC0_OUTPUT;
  
  if (AC0.STATUS & AC_STATE_bm) {
    VREF.CTRLA = VREF_DAC0REFSEL_1V5_gc;
    AC0_OUTPUT = HIGH;
  }
  else {
    VREF.CTRLA = VREF_DAC0REFSEL_2V5_gc;
    AC0_OUTPUT = LOW;
  }
  if (ProgFlag == 0)//working mode
  {
    //flag for normal working mode
    //    Serial.println("PRGM_PIN low");

    switch (Mode)
    {
      case 1:
        {
          if (AC0_OUTPUT) //(AC0.STATUS & AC_STATE_bm) //(digitalRead(SIGNAL_IN) == HIGH)
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
      if (Mode > 4)Mode = 4;
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
  /*Select proper inputs for comparator*/
  AC0.MUXCTRLA = AC_MUXPOS_PIN0_gc | AC_MUXNEG_VREF_gc; /* Negative Input - Voltage Reference */

  AC0.CTRLA = AC_ENABLE_bm              /* Enable analog comparator */
              | AC_HYSMODE_50mV_gc      /* Enable hysteresis @50mV  */
              | AC_INTMODE_BOTHEDGE_gc ;  /* Any Edge */
  AC0.INTCTRL = AC_CMP_bm;              /* Analog Comparator 0 Interrupt enabled */
}



void setup() {
  //  Serial.begin(57600);
  /* Positive Input - Disable digital input buffer */
  PORTA.PIN7CTRL = PORT_ISC_INPUT_DISABLE_gc; //For analog SIGNAL_IN
  pinMode(PRGM_PIN, INPUT_PULLUP);
  pinMode(PNP_OUT, OUTPUT);
  pinMode(NPN_OUT, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(PNP_OUT, LOW);
  digitalWrite(NPN_OUT, LOW);
  AC0_init();

  Mode = EEPROM.read(EEPROM_ADDR);
  //  Serial.println(Mode);
  if ((Mode == 0xFF) || (Mode == 0x0))
  {
    EEPROM.write(EEPROM_ADDR, POP_MODE);
    Mode = EEPROM.read(EEPROM_ADDR);
    //    Serial.println(Mode);
  }
  if (digitalRead(PRGM_PIN) )
    modeDisplayed = 1;
  sei();            /*Global interrupts enabled */

}

void loop() {
  // put your main code here, to run repeatedly:

  if (digitalRead(PRGM_PIN) == LOW ) // Working mode
  {
    ProgFlag = 0;
    //    Serial.println(Mode);
    if ( modeDisplayed)
    {
      delay(2000);
      modeDisplayed = 0;
      blinkNo = Mode;

      EEPROM.write(EEPROM_ADDR, blinkNo);
      blinkNo = EEPROM.read(EEPROM_ADDR);

      //      Serial.println(blinkNo);
      while (blinkNo--) {
        digitalWrite(LED_PIN, HIGH);
        delay(250);
        digitalWrite(LED_PIN, LOW);
        delay(750);
      }
      delay(5000);
      AC0.CTRLA = AC_ENABLE_bm | AC_HYSMODE_50mV_gc | AC_INTMODE_BOTHEDGE_gc ;
      AC0_AC_vect();
    }
    else if (Prog_Trigger == 0)
    {
      Mode = EEPROM.read(EEPROM_ADDR);
      AC0.CTRLA = AC_ENABLE_bm | AC_HYSMODE_50mV_gc | AC_INTMODE_BOTHEDGE_gc ;
      AC0_AC_vect();

    }

    Prog_Trigger = 1;
  }
  else // Program mode
  {
    delay(400);
    if (digitalRead(PRGM_PIN) == HIGH )
    {
      if (Prog_Trigger)
      {
        ProgFlag = 1;
        Mode = 0;
        AC0.CTRLA = AC_ENABLE_bm | AC_HYSMODE_50mV_gc | AC_INTMODE_NEGEDGE_gc ;
        digitalWrite(PNP_OUT, LOW);
        digitalWrite(NPN_OUT, LOW);
        digitalWrite(LED_PIN, LOW);
        Prog_Trigger = 0;
      }
    }

  }

  //AC_INTMODE_NEGEDGE_gc = (0x02<<4),  /* Negative Edge */
  //AC_INTMODE_POSEDGE_gc = (0x03<<4),  /* Positive Edge */
}
