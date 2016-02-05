#include <Adafruit_NeoPixel.h>

// Import NeoPixel library to initialize pixel 
#define VALV_PUMP 7
#define VALV_CBR 6
#define VALV_ATM 5
#define PUMP	4
#define BTN 2
#define PRS_SEN 9

// Different states of the machine: Idle, Vacuum, Dwell, and Drain
enum states {IDLED=1, VACU=2, DWLL=3, DRIN=4};
Adafruit_NeoPixel ledStat = Adafruit_NeoPixel(1, 3, NEO_GRB + NEO_KHZ800);
unsigned long startTime;

// Machine of various 'states' and updating method
class Machine
{
    public:
    // Class Member Variables
	volatile int state;						// Current state of the machine
	unsigned long vacDwlTime;    					// Time limit for Vacuum and Dwell state combined
	int updateInterval; 					// Update interval for checking for changed button state
 
 	// Constructor for Machine class
  	
  	Machine(int startState, unsigned long setTime, int interval = 200)
  	{
		state = startState;
  	  	vacDwlTime = setTime;
		updateInterval = interval;
  	}
};

Machine coffee(IDLED, 30000); 
volatile int* volatile ptr = &coffee.state;

void setup() {
	Serial.begin(9600);

	pinMode(VALV_PUMP, OUTPUT);
	pinMode(VALV_CBR, OUTPUT);
	pinMode(VALV_ATM, OUTPUT);
	pinMode(PUMP, OUTPUT);

	pinMode(PRS_SEN, INPUT_PULLUP);
	pinMode(BTN, INPUT_PULLUP);
	attachInterrupt(0, changeState, FALLING);
}

long lastDebounceTime = 0;
volatile int lastState;
void changeState() {
    
    if ((millis() - lastDebounceTime) > coffee.updateInterval) {    
    	switch (coffee.state)
    	{
    		case IDLED:
    			coffee.state = VACU;
    			//	IDLED -> Vacuum
    			break;
    		case VACU:
    			coffee.state = DRIN;
    			//	Vacuum -> Drain
    			break;
    		case DWLL:
//       			coffee.state = DRIN;
    			//	Dwell -> Drain
    			break;
    		case DRIN:
    			coffee.state = IDLED;
    			//	Drain -> IDLED
    			break;
    	}
        if (lastState == coffee.state) {
            Serial.println("------");
        } else {
            ptr = &coffee.state;
            lastState = coffee.state;
        }
        
        lastDebounceTime = millis();
    }
}


void loop() {
	// Execute methods due to volatile state changes

	switch (*ptr)
	{
		case IDLED:
            Serial.println("IDLE");
			idled();
			break;

		case VACU:
            Serial.println("VACU");
			vacuum();
			break;

		case DWLL:
            Serial.println("DWLL");
			dwell();
			break;

		case DRIN:
            Serial.println("DRIN");
			drain();
			break;
	}

	if ((*ptr == VACU || *ptr == DWLL) && (millis() - startTime) > coffee.vacDwlTime) {
        Serial.println("----------------------------------------------");
		coffee.state = DRIN;
	} else if ((millis() - startTime) <= coffee.vacDwlTime && *ptr == VACU && digitalRead(PRS_SEN) == LOW) {
		coffee.state = DWLL;
	} else if ((millis() - startTime) <= coffee.vacDwlTime && *ptr == DWLL && digitalRead(PRS_SEN) == HIGH) {
		coffee.state = VACU;
	}

}

void idled() {
	ledStat.clear();
    digitalWrite(VALV_PUMP, LOW);
    digitalWrite(VALV_CBR, LOW);
    digitalWrite(VALV_ATM, LOW);
    digitalWrite(PUMP, LOW);
}

void vacuum() {
	startTime = millis();
	digitalWrite(VALV_PUMP, HIGH);
	digitalWrite(PUMP, HIGH);
    digitalWrite(VALV_CBR, LOW);
   
    ledStat.setPixelColor(0, ledStat.Color(255, 255,0));
    ledStat.show();
}

void dwell() {
	digitalWrite(VALV_PUMP, LOW);
	digitalWrite(PUMP, LOW);
	digitalWrite(VALV_CBR, HIGH);

    ledStat.setPixelColor(0, ledStat.Color(255, 0, 0));
    ledStat.show();
}

void drain() {  
	digitalWrite(VALV_PUMP, LOW);
	digitalWrite(VALV_CBR, LOW);
	digitalWrite(VALV_ATM, HIGH);

    ledStat.setPixelColor(0, ledStat.Color(0, 255,0));
    ledStat.show();
}


