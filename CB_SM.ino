// Import NeoPixel library to initialize pixel 
#define VALV_PMP 7
#define VALV_CBR 6
#define VAVL_ATM 5
#define PUMP	4
#define BTN 3
#define PRS_SEN 8

// Different states of the machine: Idle, Vacuum, Dwell, and Drain
enum states {IDLE, VACU, DWLL, DRIN}

unsigned long startTime;

// Machine of various 'states' and updating method
class Machine
{
    // Class Member Variables
	volatile int state;						// Current state of the machine
	long vacDwlTime;    					// Time limit for Vacuum and Dwell state combined
	int updateInterval; 					// Update interval for checking for changed button state
 
 	// Constructor for Machine class
  	public:
  	Machine(int startState, long setTime, int interval = 20)
  	{
		state = startState;
  	  	vacDwlTime = setTime;
		updateInterval = interval;
  	}
};

Machine coffee(IDLE, 300000); 
volatile int* volatile ptr = coffee.state;

void setup() {
	Serial.begin(9600);

	pinMode(VALV_PMP, OUTPUT);
	pinMode(VALV_CBR, OUTPUT);
	pinMode(VALV_ATM, OUTPUT);
	pinMode(PUMP, OUTPUT);

	pinMode(PRS_SEN, INPUT);
	
	pinMode(BTN, INPUT_PULLUP);
	attachINterrupt(BTN, changeState, FALLING);
}

void changeState() {
	switch (coffee.state)
	{
		case IDLE:
			coffee.state = VACU;
			//	Idle -> Vacuum
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
			coffee.state = IDLE
			//	Drain -> Idle
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
		case IDLE:
			idle();
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

void idle() {
	led.clear();
}

void vacuum() {
	startTime = millis();
	digitalWrite(VALV_PUMP, HIGH);
	led.strip(0, led.strip.Color(125, 125, 0))
	digitalWrite(PUMP, HIGH);
}

void dwell() {
	digitalWrite(VALV_PUMP, LOW);
	digitalWrite(PUMP, LOW);
	digitalWrite(VALV_CBR, HIGH);

	led.strip(0, led.Color(255, 0, 0));
}

void drain() {
	digitalWrite(VALV_PUMP, LOW);
	digitalWrite(VALV_CBR, LOW);
	digitalWrite(VALV_ATM, HIGH);

	led.strip(0, led.Color(0, 255, 0));
}


