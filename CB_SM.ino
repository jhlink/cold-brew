#include <Adafruit_NeoPixel.h>

// Import NeoPixel library to initialize pixel 
#define VALV_PUMP 7
#define VALV_CBR 6
#define VALV_ATM 5
#define PUMP	4
#define BTN 3
#define PRS_SEN 8

// Different states of the machine: Idle, Vacuum, Dwell, and Drain
enum states {IDLED, VACU, DWLL, DRIN};
Adafruit_NeoPixel ledStat = Adafruit_NeoPixel(1, 2, NEO_GRB + NEO_KHZ800);
unsigned long startTime;

// Machine of various 'states' and updating method
class Machine
{
    public:
    // Class Member Variables
	volatile int state;						// Current state of the machine
	long vacDwlTime;    					// Time limit for Vacuum and Dwell state combined
	int updateInterval; 					// Update interval for checking for changed button state
 
 	// Constructor for Machine class
  	
  	Machine(int startState, long setTime, int interval = 20)
  	{
		state = startState;
  	  	vacDwlTime = setTime;
		updateInterval = interval;
  	}
};

Machine coffee(IDLED, 300000); 
volatile int* volatile ptr = &coffee.state;

void setup() {
	Serial.begin(9600);

	pinMode(VALV_PUMP, OUTPUT);
	pinMode(VALV_CBR, OUTPUT);
	pinMode(VALV_ATM, OUTPUT);
	pinMode(PUMP, OUTPUT);

	pinMode(PRS_SEN, INPUT);
	
	pinMode(BTN, INPUT_PULLUP);
	attachInterrupt(BTN, changeState, FALLING);
}

void changeState() {
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
			coffee.state = DRIN;
			//	Dwell -> Drain
			break;
		case DRIN:
			coffee.state = IDLED;
			//	Drain -> IDLED
			break;
		default:
			break;
	}
	ptr = &coffee.state;
}


void loop() {
	// Execute methods due to volatile state changes
	

	switch (*ptr)
	{
		case IDLED:
			idled();
			ptr = NULL;
			break;

		case VACU:
			vacuum();
			ptr = NULL;
			break;

		case DWLL:
			dwell();
			ptr = NULL;
			break;

		case DRIN:
			drain();
			ptr = NULL;
			break;
	}

	if ((*ptr == VACU || *ptr == DWLL) && (millis() - startTime) > coffee.vacDwlTime) {
		coffee.state = DRIN;
	} else if ((millis() - startTime) <= coffee.vacDwlTime && *ptr == VACU && digitalRead(PRS_SEN) == HIGH) {
		coffee.state = DWLL;
	} else if ((millis() - startTime) <= coffee.vacDwlTime && *ptr == DWLL && digitalRead(PRS_SEN) == LOW) {
		coffee.state = VACU;
	}
}

void idled() {
	ledStat.clear();
}

void vacuum() {
	startTime = millis();
	digitalWrite(VALV_PUMP, HIGH);
	ledStat.setPixelColor(0, ledStat.Color(255, 255,0));
	digitalWrite(PUMP, HIGH);
}

void dwell() {
	digitalWrite(VALV_PUMP, LOW);
	digitalWrite(PUMP, LOW);
	digitalWrite(VALV_CBR, HIGH);

    ledStat.setPixelColor(0, ledStat.Color(255, 0, 0));
}

void drain() {
	digitalWrite(VALV_PUMP, LOW);
	digitalWrite(VALV_CBR, LOW);
	digitalWrite(VALV_ATM, HIGH);

    ledStat.setPixelColor(0, ledStat.Color(0, 255,0));
}


