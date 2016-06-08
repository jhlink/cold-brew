#include <FiniteStateMachine.h>
#include <Adafruit_NeoPixel.h>

#define DEBUG
#define VALV_ATM 14
#define VACU_PUMP	13
#define TACT_TCH     5
#define NEOPIX  6 

//  CAM micro switch end stops
//	When switch is hit/pressed, returns 3v3 HIGH
//		-- Normally Open
#define MICRO_SWT_A 3
#define MICRO_SWT_B 4

// Drain Motor H-Bridge
#define HBRIDGE_A 10
#define HBRIDGE_B 20

// Pressure switch transfer function
//		Vout = Vs x (0.018 x P + .92)
//		Converted formula is found at convertVoltageToPressure()
#define PRESSURE	2

#define NUM_OF_PIX 1

//  Tell compiler that functions exist, just implementated later
extern void idled();
extern void vacuum();
extern void dwell();
extern void drain();
extern void vacuumUpdate();
extern void dwellUpdate();
extern void drainUpdate();

////	Define the states within the SM

const unsigned long TIME_FOR_MTR_TO_OPEN_LID = 210;
const unsigned long PULSE_TIME = 10;
const unsigned long DISSIPATE_TIME = 100;

// Idle state
State idle_state = State(idled);			

// Vacuum state, with accompanying update function for specific behaviours and controls 
State vacu_state = State(vacuum, vacuumUpdate, NULL);	

// Dwell state, with accompanying update function for extra dwell state behaviours	
State dwll_state = State(dwell, dwellUpdate, NULL);		

// Drain state, with timeout of X seconds
State drin_state = State(drain, drainUpdate, NULL);	


FSM stateMachine = FSM(idle_state);		//	Initial SM with beginning state

unsigned long startTime;			//	Used to track times for Vacuum/Dwell timer

/********** TIMEOUT FOR SUM OF VACUUM AND DWELL TIMES **********/
//unsigned long setTimeLimit = 60000;	// Test
unsigned long setTimeLimit = 480000;		//	User defined time limit


/********** TIMEOUT FOR DRAIN TIME **********/
//unsigned long drainTimeout = 10000;   // Test
unsigned long drainTimeout = 240000;


/********** Other variables...  **********/
unsigned long stateStartTime;			//	Used to track times for entering Vacuum/Dwell states
unsigned long stateDebounceLimit = 30000;	//	Used to define state debounce time limit
unsigned long vacuDebounceLimit = 1000;    //  Used to define state debounce time limit

int buttonState;             // the current reading from the input pin
int lastButtonState = HIGH;   // the previous reading from the input pin
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 100;    // the debounce time; increase if the output flickers

// Different states of the machine: Idle, Vacuum, Dwell, and Drain
enum states {IDLED, VACU, DWLL, DRIN};
const byte NUM_OF_STATES = 4;	//	Number of total states

Adafruit_NeoPixel ledStat = Adafruit_NeoPixel(NUM_OF_PIX, NEOPIX, NEO_GRB + NEO_KHZ800);		//	Neopixel initializiation 

/*************************** COLOR FOR MODES  ***************************/
uint32_t idleLEDColor = ledStat.Color(0, 255, 0);           // Green
uint32_t vacuumLEDColor = ledStat.Color(0, 0, 255);     // Blue
uint32_t dwellLEDColor = ledStat.Color(255, 255, 255);   // White
uint32_t drainLEDColor = ledStat.Color(255, 0, 0);      // Red

void setup() {
	Serial.begin(9600);
	pinMode(DRAIN_MTR, OUTPUT);
	pinMode(VALV_ATM, OUTPUT);
	pinMode(VACU_PUMP, OUTPUT);
	pinMode(PRS_SEN, INPUT);
	pinMode(TACT_TCH, INPUT);

	//	Defunct
	//pinMode(MECH_SWT, INPUT_PULLUP);

	pinMode(MICRO_SWT_A, INPUT);
	pinMode(MICRO_SWT_B, INPUT);

	pinMode(HBRIDGE_A, OUTPUT);
	pinMode(HBRIDGE_B, OUTPUT);

	pinMode(PRESSURE, INPUT);
	ledStat.begin();
}

//	Keeps track of button presses to direct button states
//	Value only exists within this function; Value exists between iterations
static int buttonPresses = 0; 

void loop() {

    // read the state of the switch into a local variable:
    int reading = digitalRead(TACT_TCH);
    
    // If the switch changed, due to noise or pressing:
    if (reading != lastButtonState) {
        // reset the debouncing timer
        lastDebounceTime = millis();
    }
    
    if ((millis() - lastDebounceTime) > debounceDelay) {
        // if the button state has changed:
        if (reading != buttonState) {
          buttonState = reading;
    
          // only toggle the LED if the new button state is HIGH
          if (buttonState == LOW) {
            //    Buttons used to manipulate states in SM
            
        
            //  Increment buttonPresses and constrain it to [0, NUM_OF_STATES - 1]
            buttonPresses = ++buttonPresses % NUM_OF_STATES; 
            
            Serial.print("This is what: ");
            Serial.println(buttonPresses);
            switch (buttonPresses){
                case IDLED:
                	Serial.println("IDLE");
					closeArmIntoClosedLidState();
               		stateMachine.transitionTo(idle_state);
                	break;
        
                case VACU:
                	Serial.println("VACU");

					closeArmIntoClosedLidState();

                   	//  Begin timer for total time between Vacuum and Dwell states
                   	startTime = millis(); 
					stateStartTime = millis();

					//	Ensure next button press will lead to DRAIN state.
					buttonPresses = 2; 

                    stateMachine.transitionTo(vacu_state);
                    break;
        
                case DWLL:
                    Serial.println("DWLL");
                    stateMachine.transitionTo(drin_state);
                    break;
        
                case DRIN:
                	Serial.println("DRIN");
					moveArmIntoDrainOpenState();
					stateStartTime = millis();
                	stateMachine.transitionTo(drin_state);
                	break;
            }
          }
        }
    }

#ifdef DEBUG
//    if ((millis() % 1000) == 0) {
//        Serial.print("This is pressure switch");
//        Serial.println(digitalRead(PRS_SEN));
//    }
#endif

    lastButtonState = reading;

	//	Updates the SM for every loop -- APPLICATION CRITICAL
    stateMachine.update();
}

void moveArmIntoDrainOpenState() {
	//	At switch state ON (When CAM is pushing against 
	//		push button) CAM is in closed state.
	//		Default value is OFF, or logic high; switch
	//		uses input pullup.
	//static unsigned long currentTime = millis();
	int microBSwtResult = digitalRead(MICRO_SWT_B);
	Serial.println("DRAINING -- ARM OPENING");
    Serial.print("This is switch value: ");
    Serial.println(microBSwtResult);
	while (microBSwtResult == HIGH) {
		moveForwardDrainMotorSpeed(PULSE_TIME);
		stopDrainMotor();

		//digitalWrite(DRAIN_MTR, HIGH);	
		//delay(PULSE_TIME);
		//digitalWrite(DRAIN_MTR, LOW);
		//delay(DISSIPATE_TIME);
		microBSwtResult = digitalRead(MICRO_SWT_B);
		if (microBSwtResult == LOW) {
			Serial.println("THIS IS EVAL TO TRUE");
		}
	}
    Serial.print("This is switch value 2: ");
    Serial.println(digitalRead(MICRO_SWT_B));

	stopDrainMotor();
	//digitalWrite(DRAIN_MTR, LOW);

	//digitalWrite(DRAIN_MTR, HIGH);
    //Serial.println("PREPARING TO DRAIN -- ARM MOVING");
	//delay(TIME_FOR_MTR_TO_OPEN_LID);
	//digitalWrite(DRAIN_MTR, LOW);
}

// Encapsulate microswitches into closeARmIntoClosedLidState
void closeArmIntoClosedLidState() {
	//	At switch state ON (When CAM is pushing against 
	//		push button) CAM is in closed state.
	//		Default value is OFF, or logic high; switch
	//		uses input pullup.
	//static unsigned long currentTime = millis();
	int microASwtResult = digitalRead(MICRO_SWT_A);
	Serial.println("POST DRAIN -- ARM CLOSING");
    Serial.print("This is switch value: ");
    Serial.println(microASwtResult);
	while (microASwtResult == HIGH) {
		moveReverseDrainMotorSpeed(PULSE_TIME);
		stopDrainMotor();

		//digitalWrite(DRAIN_MTR, HIGH);	
		//delay(PULSE_TIME);
		//digitalWrite(DRAIN_MTR, LOW);
		//delay(DISSIPATE_TIME);
		microASwtResult = digitalRead(MICRO_SWT_A);
		if (microASwtResult == LOW) {
			Serial.println("THIS IS EVAL TO TRUE");
		}
	}
    Serial.print("This is switch value 2: ");
    Serial.println(digitalRead(MICRO_SWT_B));

	stopDrainMotor();

	//digitalWrite(DRAIN_MTR, LOW);
}

float convertVoltageToPressure(int inputValue) {
    //  Convert analogValue to voltage.
    float voltageValue = inputValue * 3.3/1024
    
    //  Convert Vout voltage divider value into Vin value. 
    float result = voltageValue * 3/2.0;

    //  Formula is as follows
    //      Vout = Vs * (0.018  * P + 0.92)  +- (PressureError * TempMultiplier * .018 * Vs)
    //      Vs = 5.0 += 0.25V

    float finalResult = (result - (.92 * 5.0)) / (.018 * 5.0);
    return finalResult;
}

void setPixelRingColor(uint32_t val) {
	for (int i = 0; i < NUM_OF_PIX; i++) {
		ledStat.setPixelColor(i, val);
	}
	ledStat.show();
}	

void stopDrainMotor() { 
	analogWrite(HBRIDGE_A, 0);
	analogWrite(HBRIDGE_B, 0);
}

void moveForwardDrainMotorSpeed(int speed) {
	analogWrite(HBRIDGE_A, speed);
	analogWrite(HBRIDGE_B, 0);
}


void moveReverseDrainMotorSpeed(int speed) {
	analogWrite(HBRIDGE_A, 0);
	analogWrite(HBRIDGE_B, speed);
}

//	Pump is turned off first, pump valve closes, and finally the atm valve
//		closes. LED status is cleared. 
void idled() {
	digitalWrite(VACU_PUMP, LOW);

//	Due to H-Bridge, separating out motor functions for simplicity.
//	digitalWrite(DRAIN_MTR, LOW);
	stopDrainMotor();

	digitalWrite(VALV_ATM, LOW);

	setPixelRingColor(idleLEDColor);
}

//	The pump valve opens, then the pump turns on. Atm valve is ensured
//		to be closed due to necessary safety redundancy. Displays 
//		Yellow on LED status.
void vacuum() {
	digitalWrite(VACU_PUMP, HIGH);
	digitalWrite(VALV_ATM, LOW);

//	Due to H-Bridge, separating out motor functions for simplicity.
//	digitalWrite(DRAIN_MTR, LOW);
	stopDrainMotor();
	   
	setPixelRingColor(vacuumLEDColor);
}

//	Keeps track of pressure and transitions to Dwell state if 
//		pressure threshold is met. If time set by setTimeLimit
//		is met, transition immediately to the Drain state.
void vacuumUpdate() {
	//	Keep track of time. Kick into Drain state when time is up. 
	//		Ensures timer has priority over pressure.
	if ((millis() - startTime) > setTimeLimit) {
		stateStartTime = millis();

		//	Ensure that next button press in DRAIN state will lead to IDLE
		buttonPresses = 3;

        moveArmIntoDrainOpenState();
       	stateMachine.immediateTransitionTo(drin_state);
   	} else if ((digitalRead(PRS_SEN) == LOW) && ((millis() - stateStartTime) > vacuDebounceLimit)) {
    	//	If the pressure is at or above the calibrated sensor threshold
    	//		the pin will read LOW, setting immediate transition to
    	//		the Dwell state. PRS_SEN is NO.
        Serial.print("Vacuum Update:  \t");
        Serial.println((millis() - stateStartTime));

		//  Resets state start time for next state.
        stateStartTime = millis();

		//	Ensure button kicks into DRAIN state if button is pressed
		buttonPresses = 2;

    	stateMachine.immediateTransitionTo(dwll_state);
	}
}

//	Pump is first turned off, then the valve pump is turned off. 
//		Atm valve is ensured to be closed. Displays Red on LED status.
void dwell() {
	digitalWrite(VACU_PUMP, LOW);
   	digitalWrite(VALV_ATM, LOW);


//	Due to H-Bridge, separating out motor functions for simplicity.
//	digitalWrite(DRAIN_MTR, LOW);
	stopDrainMotor();
	    
	setPixelRingColor(dwellLEDColor);
}

//	If pressure fall threshold, SM transitions immediately to Vacuum state.
//		Similar timer exists as in vacuumUpdate.
void dwellUpdate() {
	//	Keep track of time. Kick into Drain state when time is up. 
	//		Ensures timer has priority over pressure.
	if ((millis() - startTime) > setTimeLimit) {
		stateStartTime = millis();

		//	Ensure that next button press in DRAIN state will lead to IDLE
		buttonPresses = 3;

        moveArmIntoDrainOpenState();
		stateMachine.immediateTransitionTo(drin_state);
	} else if ((digitalRead(PRS_SEN) == HIGH) && ((millis() - stateStartTime) > stateDebounceLimit)) {
    	//	If pressure is below set pressure threshold, return to 
    	//		Vacuum state to repressurize. 
        Serial.print("Dwell Update: \t");
        Serial.println((millis() - stateStartTime));

        stateStartTime = millis();
		stateMachine.immediateTransitionTo(vacu_state);
	}
} 


//	Pump is turned off first, then the pump valve closes. The atm valve
//		opens, and displays Green as the LED status.
void drain() {  
	// Close pump valve, turn off pump, and open the atmosphere valve
	//	to equalize with external pressure.
	digitalWrite(VACU_PUMP, LOW);
	//digitalWrite(DRAIN_MTR, LOW);
	digitalWrite(VALV_ATM, HIGH);

	setPixelRingColor(drainLEDColor);
}


void drainUpdate() {
	if((millis() - stateStartTime) >  drainTimeout) {
		closeArmIntoClosedLidState();
		stateMachine.transitionTo(idle_state);
	}
}

