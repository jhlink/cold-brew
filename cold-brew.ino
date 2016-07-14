#include <FastLED.h>
#include <FiniteStateMachine.h>
#include <SimbleeForMobile.h>

// include newlib printf float support (%f used in sprintf below)
asm(".global _printf_float");

#define DEBUG
#define VALV_ATM 14
#define VACU_PUMP	13
#define TACT_TCH     5
#define NEOPIX  6

//  CAM micro switch end stops
//	When switch is hit/pressed, returns 3v3 HIGH
//		-- Normally Open

// This is the switch to detect if the switch has open
#define MICRO_SWT_A 3

// This is the switch to detect if the switch ha closed
#define MICRO_SWT_B 4

// Drain Motor H-Bridge
#define HBRIDGE_A 10
#define HBRIDGE_B 20

// Pressure switch transfer function
//		Vout = Vs x (0.018 x P + .92)
//		Converted formula is found at convertVoltageToPressure()
#define PRS_SEN	2

#define NUM_OF_PIX 12

signed int TARGET_PRESSURE = -45;

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
//unsigned long drainTimeout = 30000;   // Test
unsigned long drainTimeout = 240000;
unsigned long repressurizeTime = 1000;


/********** Other variables...  **********/
unsigned long stateStartTime;			//	Used to track times for entering Vacuum/Dwell states
unsigned long stateDebounceLimit = 30000;	//	Used to define state debounce time limit
unsigned long vacuDebounceLimit = 1000;    //  Used to define state debounce time limit

int buttonState;             // the current reading from the input pin
int lastButtonState = HIGH;   // the previous reading from the input pin
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 100;    // the debounce time; increase if the output flickers

//	Keeps track of button presses to direct button states
//	Value only exists within this function; Value exists between iterations
static int buttonPresses = 0; 

// Different states of the machine: Idle, Vacuum, Dwell, and Drain
enum states {IDLED, VACU, DWLL, DRIN};
const byte NUM_OF_STATES = 4;	//	Number of total states

//	Simblee asset IDs
uint8_t	pressureDataLabel;
uint8_t brewTimeField;
uint8_t vacuumSwitch;
uint8_t saveButton;
uint8_t saveButtonConfirmState;

bool buttonTest = false;

//	Simblee temporry values for event
//		NOTE: Values from assets are only sent through events when the assets
//			themselves change. Reducing what data needs to be sent, but I don't think
//			data can be requested over the BLE line.
unsigned long tempBrewTimeLimit = 0;

CRGB ledStat[NUM_OF_PIX];

/*************************** COLOR FOR MODES  ***************************/
CRGB::HTMLColorCode idleLEDColor = CRGB::Green;          // Green
CRGB::HTMLColorCode vacuumLEDColor = CRGB::Blue;    // Blue
CRGB::HTMLColorCode dwellLEDColor = CRGB::White;   // White
CRGB::HTMLColorCode drainLEDColor = CRGB::Red;   // Red

void setup() {
	Serial.begin(9600);
	//	pinMode(DRAIN_MTR, OUTPUT);
	pinMode(VALV_ATM, OUTPUT);
	pinMode(VACU_PUMP, OUTPUT);
	pinMode(PRS_SEN, INPUT);
	pinMode(TACT_TCH, INPUT_PULLUP);

	//	Defunct
	//pinMode(MECH_SWT, INPUT_PULLUP);

	pinMode(MICRO_SWT_A, INPUT_PULLUP);
	pinMode(MICRO_SWT_B, INPUT_PULLUP);

	pinMode(HBRIDGE_A, OUTPUT);
	pinMode(HBRIDGE_B, OUTPUT);

	//	pinMode(PRESSURE, INPUT);
	//    ledStat.begin();
	FastLED.addLeds<NEOPIXEL, NEOPIX>(ledStat, NUM_OF_PIX);
	FastLED.setBrightness(150);

	SimbleeForMobile.advertisementData = "prismav2";

	SimbleeForMobile.domain = "prisma.firstbuild.com";

	// establish a baseline to use the cache during development to bypass uploading
	// the image each time
	SimbleeForMobile.baseline = "Jul14";

	// start SimbleeForMobile
	SimbleeForMobile.begin();
}

void resetSaveButtonConfirmLabel() {
    // Providing update to what the value of the Pressure is in kPa
    //  SimbleForMobile.updateValue(convertVoltageToPressure(analogRead(PRS_SEN)));
    SimbleeForMobile.updateText(saveButtonConfirmState, "READY");
}


void ignoreSaveButtonConfirmLabel() {
    // Providing update to what the value of the Pressure is in kPa
    //  SimbleForMobile.updateValue(convertVoltageToPressure(analogRead(PRS_SEN)));
    SimbleeForMobile.updateText(saveButtonConfirmState, "BREW BABY BREW");
}


void setPixelRingColor(CRGB val) {
	for (int i = 0; i < NUM_OF_PIX; i++) {
		ledStat[i] = val;
	}
	FastLED.show();
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

void moveArmIntoDrainOpenState() {
	//	At switch state ON (When CAM is pushing against 
	//		push button) CAM is in closed state.
	//		Default value is OFF, or logic high; switch
	//		uses input pullup.
	//static unsigned long currentTime = millis();
	int microASwtResult = digitalRead(MICRO_SWT_A);
	Serial.println("DRAINING -- ARM OPENING");
	Serial.print("This is switch open: ");
	Serial.println(microASwtResult);
	while (microASwtResult == HIGH) {
		moveForwardDrainMotorSpeed(255);
		//		stopDrainMotor();

		//digitalWrite(DRAIN_MTR, HIGH);	
		//delay(PULSE_TIME);
		//digitalWrite(DRAIN_MTR, LOW);
		//delay(DISSIPATE_TIME);
		microASwtResult = digitalRead(MICRO_SWT_A);
		if (microASwtResult == LOW) {
			Serial.println("THIS IS EVAL TO TRUE");
		}
	}
	Serial.print("This is switch close 2: ");
	Serial.println(digitalRead(MICRO_SWT_A));

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
	int microBSwtResult = digitalRead(MICRO_SWT_B);
	Serial.println("POST DRAIN -- ARM CLOSING");
	Serial.print("This is switch close: ");
	Serial.println(microBSwtResult);
	while (microBSwtResult == HIGH) {
		moveReverseDrainMotorSpeed(255);
		//		stopDrainMotor();

		//digitalWrite(DRAIN_MTR, HIGH);	
		//delay(PULSE_TIME);
		//digitalWrite(DRAIN_MTR, LOW);
		//delay(DISSIPATE_TIME);
		microBSwtResult = digitalRead(MICRO_SWT_B);
		if (microBSwtResult == LOW) {
			Serial.println("THIS IS EVAL TO TRUE");
		}
	}
	Serial.print("This is switch close 2: ");
	Serial.println(digitalRead(MICRO_SWT_B));

	stopDrainMotor();

	//digitalWrite(DRAIN_MTR, LOW);
}

float convertVoltageToPressure(int inputValue) {
	//  Convert analogValue to voltage.
	float voltageValue = inputValue * 3.3/1024;

	//  Convert Vout voltage divider value into Vin value. 
	float result = voltageValue * 3/2.0;

	//  Formula is as follows
	//      Vout = Vs * (0.018  * P + 0.92)  +- (PressureError * TempMultiplier * .018 * Vs)
	//      Vs = 5.0 += 0.25V

	float finalResult = (result - (.92 * 5.0)) / (.018 * 5.0);
	return finalResult;
}

//	Pump is turned off first, pump valve closes, and finally the atm valve
//		closes. LED status is cleared. 
void idled() {
    
    closeArmIntoClosedLidState();
	digitalWrite(VACU_PUMP, LOW);
	digitalWrite(VALV_ATM, LOW);

	//	Due to H-Bridge, separating out motor functions for simplicity.
	//	digitalWrite(DRAIN_MTR, LOW);
	stopDrainMotor();

	setPixelRingColor(idleLEDColor);
}

//	The pump valve opens, then the pump turns on. Atm valve is ensured
//		to be closed due to necessary safety redundancy. Displays 
//		Yellow on LED status.
void vacuum() {
	digitalWrite(VACU_PUMP, HIGH);
	digitalWrite(VALV_ATM, HIGH);

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

	unsigned long currentTime = millis();
	float pressure_in_kpa = convertVoltageToPressure(analogRead(PRS_SEN));
	Serial.print("This is the first test: ");
	Serial.println(pressure_in_kpa <= TARGET_PRESSURE);

	Serial.print("This is the second test: ");
	Serial.println((currentTime - stateStartTime) > vacuDebounceLimit);
	if ((currentTime - startTime) > setTimeLimit) {
		stateStartTime = currentTime;

		//	Ensure that next button press in DRAIN state will lead to IDLE
		buttonPresses = 3;

		stateMachine.immediateTransitionTo(drin_state);
	} else if ((pressure_in_kpa <= TARGET_PRESSURE) && ((currentTime - stateStartTime) > vacuDebounceLimit)) {
		//	If the pressure is at or above the calibrated sensor threshold
		//		the pin will read LOW, setting immediate transition to
		//		the Dwell state. PRS_SEN is NO.
		Serial.print("Vacuum Update:  \t");
		Serial.println((currentTime - stateStartTime));

		//  Resets state start time for next state.
		stateStartTime = currentTime;

		//	Ensure button kicks into DRAIN state if button is pressed
		buttonPresses = 2;

		stateMachine.immediateTransitionTo(dwll_state);
	}
}

//	Pump is first turned off, then the valve pump is turned off. 
//		Atm valve is ensured to be closed. Displays Red on LED status.
void dwell() {
	digitalWrite(VACU_PUMP, LOW);
	digitalWrite(VALV_ATM, HIGH);

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
	unsigned long currentTime = millis();
	float pressure_in_kpa = convertVoltageToPressure(analogRead(PRS_SEN));
	if ((currentTime- startTime) > setTimeLimit) {
		stateStartTime = currentTime;

		//	Ensure that next button press in DRAIN state will lead to IDLE
		buttonPresses = 3;

		stateMachine.immediateTransitionTo(drin_state);
	} else if ((pressure_in_kpa > TARGET_PRESSURE) && ((currentTime - stateStartTime) > stateDebounceLimit)) {
		//	If pressure is below set pressure threshold, return to 
		//		Vacuum state to repressurize. 
		Serial.print("Dwell Update: \t");
		Serial.println(currentTime - stateStartTime);

		stateStartTime = currentTime;
		stateMachine.immediateTransitionTo(vacu_state);
	}
} 

//	Pump is turned off first, then the pump valve closes. The atm valve
//		opens, and displays Green as the LED status.
void drain() {  
	// Close pump valve, turn off pump, and open the atmosphere valve
	//	to equalize with external pressure.
	digitalWrite(VACU_PUMP, LOW);
	digitalWrite(VALV_ATM, LOW);

	setPixelRingColor(drainLEDColor);
}


void drainUpdate() {
	unsigned long currentTime = millis();
	if ((currentTime - stateStartTime) >  drainTimeout) {
		closeArmIntoClosedLidState();
        resetSaveButtonConfirmLabel();
		stateMachine.transitionTo(idle_state);
	}

	//	Creates delay between atmo valve and vacuum pump.
	if ((currentTime - stateStartTime) > repressurizeTime) {
		moveArmIntoDrainOpenState();
	}
}



void update() {
  // Providing update to what the value of the Pressure is in kPa
  //  SimbleForMobile.updateValue(convertVoltageToPressure(analogRead(PRS_SEN)));

  char buf[16];

    static unsigned long stateStartTime = millis();
    //  Creates staggers write to app.
    if ((millis() - stateStartTime) > 250) {
      float pressureValue = convertVoltageToPressure(analogRead(PRS_SEN));
      sprintf(buf, "%.02f", pressureValue);
      stateStartTime = millis();
      SimbleeForMobile.updateText(pressureDataLabel, buf);
    }


  //  Need to provide similar update for brew time.
  if (!stateMachine.isInState(idle_state)) {
      memset(buf, 0, sizeof(buf));
//      sprintf(buf, "%d min.", setTimeLimit);
      SimbleeForMobile.updateValue(brewTimeField, setTimeLimit / 1000);
  }
}

void loop() {

	// read the state of the switch into a local variable:
	int reading = digitalRead(TACT_TCH);

	unsigned long currentTime = millis();

	// If the switch changed, due to noise or pressing:
	if (reading != lastButtonState) {
		// reset the debouncing timer
		lastDebounceTime = currentTime;
		//        Serial.println("testing buttonState");
	}

	if ((currentTime - lastDebounceTime) > debounceDelay) {
		// if the button state has changed:
		//        Serial.println("passing time test");
		if (reading != buttonState) {
			//           Serial.println("passing button test");
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
                        resetSaveButtonConfirmLabel();
						stateMachine.transitionTo(idle_state);
						break;

					case VACU:
						Serial.println("VACU");

						closeArmIntoClosedLidState();
                        ignoreSaveButtonConfirmLabel();

						//  Begin timer for total time between Vacuum and Dwell states
						startTime = currentTime; 
						stateStartTime = currentTime;

						//  Ensure next button press will lead to DRAIN state.
						buttonPresses = 2; 

						stateMachine.transitionTo(vacu_state);
						break;

					case DWLL:
						Serial.println("DWLL");
						stateMachine.transitionTo(drin_state);
						break;

					case DRIN:
						Serial.println("DRIN");
						stateStartTime = currentTime;
						stateMachine.transitionTo(drin_state);
						break;
				}
			}
		}
	}

#ifdef DEBUG
	if ((millis() % 1000) == 0) {
		Serial.print("Presssure (kPa): ");
		Serial.println(convertVoltageToPressure(analogRead(PRS_SEN)));
    

		//        Serial.print("MicroSwitch A : ");
		//        Serial.println(digitalRead(MICRO_SWT_A));
		//
		//        Serial.print("MicroSwitch B : ");
		//        Serial.println(digitalRead(MICRO_SWT_B));
		//
		//        Serial.print("Tactile: ");
		//        Serial.println(digitalRead(TACT_TCH));

	}
#endif

	lastButtonState = reading;

	//  Updates the SM for every loop -- APPLICATION CRITICAL
	stateMachine.update();


	if (SimbleeForMobile.updatable) {
		update();
	}

	SimbleeForMobile.process();
}

void updateSaveButtonConfirmLabel() {
	//	Notifying user of change in variable values.
	if (stateMachine.isInState(idle_state) || buttonTest) {
        buttonTest = false;
		setTimeLimit = tempBrewTimeLimit * 1000;
		Serial.print("This is new time ");
        Serial.println(setTimeLimit / 1000);
		SimbleeForMobile.updateText(saveButtonConfirmState, "SAVED TO MEMORY");
	} else {
		SimbleeForMobile.updateText(saveButtonConfirmState, "ERR -- NOT IN IDLE");
	}
}

void ui() { 
	color_t darkgray = rgb(85,85,85);
	SimbleeForMobile.beginScreen(darkgray);

	SimbleeForMobile.drawText(25, 75, "Brew Time (sec.):", BLACK);
	brewTimeField = SimbleeForMobile.drawTextField(125, 70, 150, setTimeLimit / 1000, "INPUT");

	//	Drain time updating feature
	//	SimbleeForMobile.drawText(25, 71, "Drain Time:", BLACK);
	//	text = SimbleeForMobile.drawTextField(100, 71, 50, "");

	SimbleeForMobile.drawText(25, 150, "Pressure:", BLACK);
	pressureDataLabel = SimbleeForMobile.drawText(125, 150, "VALUE", BLACK);

//	//	Probably will disable this for this rev.
//	SimbleeForMobile.drawText(25, 200, "(NOT IMPLEMENTED) Vacuum:", BLACK);
//	vacuumSwitch = SimbleeForMobile.drawSwitch(100, 230);

	saveButton = SimbleeForMobile.drawButton(75, 300, 200, "Save to Device");
	saveButtonConfirmState = SimbleeForMobile.drawText(75, 250, "READY", BLUE, 20);

    SimbleeForMobile.setEvents(saveButton, EVENT_RELEASE);

	SimbleeForMobile.endScreen();

	update();
}


void handleTextFieldEvents(event_t &event)
{
  if(event.id == brewTimeField) {
//        Serial.println(event.value);
        tempBrewTimeLimit = event.value;
  } 
}

void handleButtonScreenEvents(event_t &event)
{
  if(event.id == saveButton && event.type == EVENT_RELEASE) {
//        buttonTest = true; 
        updateSaveButtonConfirmLabel();
  } 
}

void ui_event(event_t &event)
{
	handleTextFieldEvents(event);
	handleButtonScreenEvents(event);
}


//	This will be a helper in code refactoring later to simplying some code blocks.
//void saveToDevice() {
//	setTimeLimit;
//	drainTimeout;
//	TARGET_PRESSURE;
//	vacuumState;
//}
