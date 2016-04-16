#include <Adafruit_NeoPixel.h>

#define NEOPIX  5
#define MECH_SWT 9
#define DRAIN_MTR 8
#define NUM_OF_PIX 12

#define LEDTEST

unsigned long newTime = millis();

const unsigned long TIME_FOR_MTR_TO_OPEN_LID = 210;

Adafruit_NeoPixel ledStat = Adafruit_NeoPixel(NUM_OF_PIX, NEOPIX, NEO_GRB + NEO_KHZ800);        //  Neopixel initializiation 

/*************************** COLOR FOR MODES  ***************************/
uint32_t idleLEDColor = ledStat.Color(0, 255, 0);           // Green
uint32_t vacuumLEDColor = ledStat.Color(0, 0, 255);     // Blue
uint32_t dwellLEDColor = ledStat.Color(255, 255, 255);   // White
uint32_t drainLEDColor = ledStat.Color(255, 0, 0);      // Red

void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
    pinMode(DRAIN_MTR, OUTPUT);
    pinMode(MECH_SWT, INPUT_PULLUP);
    ledStat.begin();

}

void loop() {
//    static unsigned long oldTime = millis();
//    digitalWrite(8, HIGH);


#ifdef  LEDTEST
//    delay(4000);
//    setPixelRingColor(idleLEDColor);
//    delay(4000);
    setPixelRingColor(vacuumLEDColor);
    delay(4000);
    setPixelRingColor(dwellLEDColor);
    delay(4000);
    setPixelRingColor(drainLEDColor);
    delay(4000);
#endif


#ifdef CAMTEST
    delay(5000);
    moveArmIntoDrainOpenState();
    delay(5000);
    closeArmIntoClosedLidState();

    Serial.println(digitalRead(MECH_SWT));
    delay(500);
#endif
   
  // put your main code here, to run repeatedly:
//    if ((digitalRead(9) == LOW) && ((newTime - oldTime) > 3000)) {
//        Serial.println("MOVING");
//        digitalWrite(8, HIGH);
//        delay(50);
//        digitalWrite(8, LOW);
//        newTime = millis();
//    }
}

void setPixelRingColor(uint32_t val) {
    for (int i = 0; i < NUM_OF_PIX; i++) {
        ledStat.setPixelColor(i, val);
    }
    ledStat.show();
}   
void moveArmIntoDrainOpenState() {
    digitalWrite(DRAIN_MTR, HIGH);
    Serial.println("PREPARING TO DRAIN -- ARM MOVING");
    delay(TIME_FOR_MTR_TO_OPEN_LID);
    digitalWrite(DRAIN_MTR, LOW);
}

void closeArmIntoClosedLidState() {

    //  At switch state ON (When CAM is pushing against 
    //      push button) CAM is in closed state.
    //      Default value is OFF, or logic high; switch
    //      uses input pullup.
    const unsigned long PULSE_TIME = 10;
    const unsigned long DISSIPATE_TIME = 100;
    //static unsigned long currentTime = millis();
    int mechSwitchResult = digitalRead(MECH_SWT);
    Serial.println("POST DRAIN -- ARM CLOSING");
    Serial.print("This is switch value: ");
    Serial.println(mechSwitchResult);
    while (mechSwitchResult == HIGH) {
        digitalWrite(DRAIN_MTR, HIGH);  
        delay(PULSE_TIME);
        digitalWrite(DRAIN_MTR, LOW);
        delay(DISSIPATE_TIME);
        mechSwitchResult = digitalRead(MECH_SWT);
        if (mechSwitchResult == LOW) {
            Serial.println("THIS IS EVAL TO TRUE");
        }
    }
    Serial.print("This is switch value 2: ");
    Serial.println(digitalRead(MECH_SWT));
    digitalWrite(DRAIN_MTR, LOW);
}
