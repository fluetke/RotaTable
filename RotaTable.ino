#include <MenuBackend.h>  //from http://www.arduino.cc/playground/uploads/Profiles/MenuBackend_1-4.zip
#include <LiquidCrystal.h>

//Define pins for easier reference in the code
#define SELECT_PIN 2              // Select Button
#define INTERRUPT_SELECT 0        // Interrupt for select
#define ROTATE_PIN_L 3            // RotaryEncoder Pin A
#define ROTATE_PIN_R 7            // RotaryEncoder Pin B
#define INTERRUPT_MOVE 1          // Interrupt for Rotary Encoder movements
#define SENSOR_PIN 6              // Plate sensor for 360deg rotation

#define SHUTTER_PIN A0            // Shutter release pin for external camera
#define MOTOR_DIR 4               // direction pin of stepper motor driver
#define MOTOR_STEP 5              // stepping pin of stepper motor driver
#define MOTOR_STEPS 200           // number of steps of motor for full rotation

//Define menu interaction related constants
#define RESOLUTION_BUFFER 10      // rotary encoder resolution tolerance //TODO: find better solution to rotary encoder sensitivity
#define DISPLAY_CHARS 16          // number of characters per line of LC-Display
#define DISPLAY_ROWS 2            // number of lines of LC-Display

//vars used in interrupts for menu handling
volatile boolean SELECT_FIRED = false;  // true if select button has been pressed
volatile boolean MOVE_UP = false;       // true if rotary encoder has been turned CCW
volatile boolean KNOB_TURNED = false;   // true if rotary encoder has been turned
volatile int REDUCE_RESOLUTION = 0;     // counts up or down until +/-10 to reduce encoder resolution

int STEPS_TO_FULL_ROTATION = 9800;   //count the amount of steps of the small stepper until the scanplate has fully rotated 
int BAUD_IDX = 5;			//Speed to communicate with
boolean DEBUG = false;            //set this to true to enable serial println of debug msgs
int PLATE_RPM = 1;                //rpm for scanplate
int PLATE_ANGLE = 45;            //angle for rotation steps 
boolean INSETTINGS = false;       //state of settings menu
int SHUTTER_IDX = 4;

// store fixed options for shutter speed and baudrate
int BAUDRATES[]={ 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 31250 }; //available baud-rates
float SHUTTER_MSECS[]={0.25, 0.5, 1 , 2, 4, 8, 16.67, 33.33, 66.67, 125, 250, 500, 1000};   //shutter values in milliseconds
String SHUTTER_NAMES[]={"1/4000", "1/2000", "1/1000", "1/500", "1/250", "1/125", "1/60", "1/30", "1/15", "1/8", "1/4", "1/2", "1"}; //names of shutter values

//Init LC-Display
LiquidCrystal lcd(13,12,11,10,9,8);

//Create MenuStructure
MenuBackend menu = MenuBackend(menuUseEvent, menuChangeEvent);
MenuItem david3D = MenuItem("David3D");
  MenuItem david_back = MenuItem("Back");
  MenuItem sls_mode = MenuItem("Structured Light Scan");
    MenuItem sls_back = MenuItem("Back");
    MenuItem sls_angle = MenuItem("Angle");
    MenuItem sls_start = MenuItem("Start SLS Scan");
  MenuItem motor_mode = MenuItem("Motor Scan");
    MenuItem motor_back("Back");
    MenuItem motor_start = MenuItem("Start Motor Scan");
MenuItem photogramm = MenuItem("Photogrammetry");
  MenuItem photog_back = MenuItem("Back");
  MenuItem photog_angle = MenuItem("Angle");
  MenuItem photog_timing = MenuItem("Timing");
  MenuItem photog_start = MenuItem("Start Photo Scan");
MenuItem contRot = MenuItem("Continuous Rotation");
  MenuItem contRot_back("Back");
  MenuItem contRot_rpm = MenuItem("RPM");
  MenuItem contRot_start = MenuItem("Start Scan");
MenuItem settings = MenuItem("Settings");
  MenuItem settings_back = MenuItem("Back");
  MenuItem baud_menu = MenuItem("Baudrate");
  MenuItem rpm_setting = MenuItem("Plate RPM");
  MenuItem calibration = MenuItem("Calibrate Disk");

//Link menu entries
void menuSetup() {
  menu.getRoot().add(david3D);
  //setup SLS menu with sls and motor mode
  david3D.addBefore(settings);
  david3D.addRight(sls_mode);
  sls_mode.addBefore(david_back);
  sls_mode.addAfter(motor_mode);
  motor_mode.addAfter(david_back);
  david_back.addAfter(sls_mode);
  david_back.addLeft(david3D);
  
  //set sub-menus for sls_mode
  sls_mode.addRight(sls_angle);
  sls_angle.addBefore(sls_back);
  sls_angle.addAfter(sls_start);
  sls_start.addAfter(sls_back);
  sls_back.addAfter(sls_angle);
  sls_back.addLeft(sls_mode);
  
  //set sub menus for motor scan mode
  motor_mode.addRight(motor_start);
  motor_start.addBefore(motor_back);
  motor_start.addAfter(motor_back);
  motor_back.addAfter(motor_start);
  motor_back.addLeft(motor_mode);
  
  //add next main item to structure
  david3D.addBefore(settings);
  david3D.addAfter(photogramm);
  
  // add photogrammetry mode submenus
  photogramm.addRight(photog_angle);
  photog_angle.addBefore(photog_back);
  photog_angle.addAfter(photog_timing);
  photog_timing.addAfter(photog_start);
  photog_start.addAfter(photog_back);
  photog_back.addAfter(photog_angle);
  photog_back.addLeft(photogramm);
  
  // add next main item
  photogramm.addAfter(contRot);
  
  contRot.addRight(contRot_rpm);
  contRot_rpm.addBefore(contRot_back);
  contRot_rpm.addAfter(contRot_start);
  contRot_start.addAfter(contRot_back);
  contRot_back.addAfter(contRot_rpm);
  contRot_back.addLeft(contRot);
  
  contRot.addAfter(settings);
  settings.addRight(baud_menu);
  baud_menu.addBefore(settings_back);
  baud_menu.addAfter(rpm_setting);
  rpm_setting.addBefore(baud_menu);
  rpm_setting.addAfter(calibration);
  calibration.addBefore(rpm_setting);
  calibration.addAfter(settings_back);
  settings_back.addAfter(baud_menu);
  settings_back.addLeft(settings);

  settings.addAfter(david3D);
}

void motorStep(int steps, int rpm=1) {

  for (int j=0; j<steps; j++) {
      digitalWrite(MOTOR_STEP, HIGH);
      delayMicroseconds(600000/(rpm*MOTOR_STEPS));
      digitalWrite(MOTOR_STEP, LOW);
      delayMicroseconds(600000/(rpm*MOTOR_STEPS));
    }

}

//this gets called everytime a menu item is clicked
void menuUseEvent(MenuUseEvent used)
{
  //clear lcd for display of new sub-menus
  lcd.clear();
  
  //comparison agains a known item
  //go back to parent menu if possible
  if (used.item == david_back || 
      used.item == sls_back || 
      used.item == motor_back || 
      used.item == photog_back || 
      used.item == contRot_back || 
      used.item == settings_back ) 
  {
    menu.moveLeft();
  }

  //react to all submenus
  if (used.item == david3D || 
      used.item == photogramm || 
      used.item == contRot || 
      used.item == settings || 
      used.item == sls_mode || 
      used.item == motor_mode )
  {
    menu.moveRight();
  }

  if (used.item == sls_angle ||
      used.item == photog_angle) {
        INSETTINGS = true;
        SELECT_FIRED = false;
        set_angle(); 
  }
  

  if (used.item == photog_timing) {
    INSETTINGS = true;
    SELECT_FIRED = false;
    set_photog_timing();
  } 
  
  if (used.item == contRot_rpm) {
    Serial.println("SETTING RPM");
    INSETTINGS = true;
    SELECT_FIRED = false;
    set_rpm(); 
  }

  if (used.item == sls_start) {
    SELECT_FIRED = false;
    structuredLight(PLATE_ANGLE);
  } 
  
  if (used.item == motor_start) {
    SELECT_FIRED = false;
    motorScan();
  }

  if (used.item == photog_start) {
    Serial.println("Starting Photogrammetry mode!");
    photogrammetry(PLATE_ANGLE,SHUTTER_MSECS[SHUTTER_IDX]);  
  } 

  if (used.item == contRot_start) {
    continuosRotation(PLATE_RPM);
  }
  
  if (used.item == calibration) {
    calibrate();
  } 
  
  if (used.item == baud_menu) {
    INSETTINGS = true;
    SELECT_FIRED = false;
    set_baudrate();
  }

  Serial.println("End of List reached");
  //lcd.println("End of List reached");
  //menu.moveLeft();
}


/*
  This is an important function
  Here we get a notification whenever the user changes the menu
  That is, when the menu is navigated
*/
void menuChangeEvent(MenuChangeEvent changed)
{
  // clear the lcd for the new writing
  lcd.clear();

  const MenuItem* tempItem = &changed.to;
  String itemString = "";
  
  for (int i=0; i<DISPLAY_ROWS; i++) {
    lcd.setCursor(0,i);
    if (i==0) 
      itemString = "> ";
    else
      itemString = " ";

    itemString += tempItem->getName();
    lcd.print(itemString);
    tempItem = tempItem->getAfter();
  }

  if (DEBUG) {
    Serial.print("Menu change ");
    Serial.print(changed.from.getName());
    Serial.print(" ");
    Serial.println(changed.to.getName());
  }
}

//interrupt service routine to handle button presses of the rotatry encoder
void isr_select()
{
  if (!digitalRead (SELECT_PIN))
    SELECT_FIRED = true;
}

//interrupt service routine to handle rotation of encoder
void isr_move()
{
  if (digitalRead (ROTATE_PIN_L))
    MOVE_UP = !digitalRead (ROTATE_PIN_R);
  else
    MOVE_UP = digitalRead (ROTATE_PIN_R);
  KNOB_TURNED = true;
  Serial.println("KNOB TURNED");
}

//prepare the inputs and communications for maximum fancyness
void setup() {
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW); //TURN OFF STEPPER
  digitalWrite(5, HIGH);
  // put your setup code here, to run once:
  menuSetup();
  pinMode(SELECT_PIN, INPUT);
  pinMode(ROTATE_PIN_L, INPUT);
  pinMode(ROTATE_PIN_R, INPUT);
  attachInterrupt(INTERRUPT_SELECT, isr_select, FALLING);
  attachInterrupt(INTERRUPT_MOVE, isr_move, CHANGE);
  lcd.begin(DISPLAY_CHARS,DISPLAY_ROWS);
  
  Serial.begin(BAUDRATES[BAUD_IDX]);
  //calibrate();
}

void loop() {
  // put your main code here, to run repeatedly:
    if (KNOB_TURNED && !INSETTINGS)
    {
      if (MOVE_UP && (REDUCE_RESOLUTION >= RESOLUTION_BUFFER)) {
        menu.moveUp();
        REDUCE_RESOLUTION = 0;}
      else if (!MOVE_UP && (REDUCE_RESOLUTION <=-RESOLUTION_BUFFER)) {
        menu.moveDown();
        REDUCE_RESOLUTION = 0;}
      KNOB_TURNED = false;
      if(MOVE_UP)
        REDUCE_RESOLUTION++;
      else
        REDUCE_RESOLUTION--;
    }
  else if (SELECT_FIRED && !INSETTINGS)
    {
    menu.use();
    SELECT_FIRED = false;
    }  // end if fired
}


void set_rpm() {
  lcd.clear();
  lcd.print("Set RPM");
  lcd.setCursor(0,2);
  lcd.print("RPM: " + String(PLATE_RPM));
  while (!SELECT_FIRED) {
    if (KNOB_TURNED) {
      Serial.println("KNOB TURNED IN RPM");
      if (MOVE_UP) {
        if (PLATE_RPM>1) {
          PLATE_RPM--;
        }
      } else {
        if (PLATE_RPM<10) {
              PLATE_RPM++;
        }
      }
      KNOB_TURNED=false;
      lcd.clear();
      lcd.setCursor(0,2);
      lcd.print("RPM: " + String(PLATE_RPM));
    }
  }
  SELECT_FIRED=false;
  INSETTINGS = false;
}


void set_angle() {
  lcd.clear();
  lcd.print("Set Angle");
  lcd.setCursor(0,2);
  lcd.print("Angle: " + String(PLATE_ANGLE) + " deg");
   while (!SELECT_FIRED) {
    if (KNOB_TURNED) {
      if (MOVE_UP) {
        if (PLATE_ANGLE>1) {
          PLATE_ANGLE--;
        }
        
      } else {
        if(PLATE_ANGLE<360) {
          PLATE_ANGLE++;
        }
      }
      KNOB_TURNED=false;
      lcd.clear();
      lcd.setCursor(0,2);
      lcd.print("Angle: " + String(PLATE_ANGLE)  + " deg");
    }
  }
  INSETTINGS = false;
}


void set_baudrate() {
  lcd.clear();
  lcd.print("Set Baudrate");
  lcd.setCursor(0,2);
  lcd.print("Baud: " + String(BAUDRATES[BAUD_IDX]));
  while (!SELECT_FIRED) {
    if (KNOB_TURNED) {
      Serial.println("KNOB TURNED IN RPM");
      if (MOVE_UP) {
        if (BAUD_IDX>0) {
          BAUD_IDX--;
        }
      } else {
        if (BAUD_IDX<(sizeof(BAUDRATES)/sizeof(int))-1) {
              BAUD_IDX++;
        }
      }
      KNOB_TURNED=false;
      lcd.clear();
      lcd.setCursor(0,2);
      lcd.print("Baud: " + String(BAUDRATES[BAUD_IDX]));
    }
  }
  SELECT_FIRED=false;
  INSETTINGS = false;
}


void set_photog_timing() {
  lcd.clear();
  lcd.print("Set Shutterspeed");
  lcd.setCursor(0,2);
  lcd.print("Shutter: " + String(SHUTTER_NAMES[SHUTTER_IDX]) + " sec");
  while (!SELECT_FIRED) {
    if (KNOB_TURNED) {
      Serial.println("KNOB TURNED IN RPM");
      if (MOVE_UP) {
        if (SHUTTER_IDX>0) {
          SHUTTER_IDX--;
        }
      } else {
        if (SHUTTER_IDX<(sizeof(SHUTTER_NAMES)/sizeof(int))-1) {
              SHUTTER_IDX++;
        }
      }
      KNOB_TURNED=false;
      lcd.clear();
      lcd.setCursor(0,2);
      lcd.print("Shutter: " + String(SHUTTER_NAMES[SHUTTER_IDX]) + " sec"); 
    }
  }
  SELECT_FIRED=false;
  INSETTINGS = false;
}

/* CALIBRATION OF DISK
   This method steps the motor until the turntable has performed one full rotation.
   The resulting values are finally displayed to allow the user to adjust the firmware 
   settings when flashing the arduino. */
void calibrate() {
  SELECT_FIRED = false;
  // return to origin
  lcd.println("Moving to origin");
  while (digitalRead(SENSOR_PIN) == HIGH) {
    motorStep(1);
    delay(5);
  }
  lcd.clear();
  delay(50);
  
  int step_count = 0;
  int sensor_area = 0;
  
  // start spinning until one rotation is done
  lcd.println("Moving to 360 mark");
  while (true) {
    motorStep(1);
    step_count++;
    delay(10);

    // slot in table has passed the sensor when the sensor output is low 
    // and the sensor_area has not been mapped yet
    if (digitalRead(SENSOR_PIN) == HIGH && sensor_area == 0) {
       sensor_area = step_count;
    }

    // table did one full revolution when the sensor_area has been mapped 
    // and the sensor output switches to HIGH
    if (digitalRead(SENSOR_PIN) == LOW && sensor_area != 0) {
       
       STEPS_TO_FULL_ROTATION = step_count;

       while (!SELECT_FIRED) {
        lcd.clear();
        lcd.print("Fullrotation:");
        lcd.setCursor(0,2);
        lcd.print(String(STEPS_TO_FULL_ROTATION) + " steps");
        Serial.println("Fullrotation steps: " + String(STEPS_TO_FULL_ROTATION));
        delay(5000);
        lcd.clear();
        lcd.print("Steps per deg:");
        lcd.setCursor(0,2);
        lcd.print(String(STEPS_TO_FULL_ROTATION/360));
        Serial.println("Steps per degree: " + String(STEPS_TO_FULL_ROTATION/360));
        delay(5000);
        lcd.clear();
        lcd.print("Deg per step:");
        lcd.setCursor(0,2);
        lcd.print(String(360/STEPS_TO_FULL_ROTATION));
        Serial.println("Degrees per step: " + String(360/STEPS_TO_FULL_ROTATION));
        delay(5000);
       }
       Serial.println("Select Button Pressed");
       lcd.clear();
       break;
    }
  }
}

/* Photogrammetry mode, rotates the plate and triggers the camera afterwards */
void photogrammetry(float angle, float timing) {
  float rotatedAngles = 0.0;
  while(rotatedAngles <= 360) { //TODO: change break condition
    lcd.clear();
    lcd.print("Rotating table");
    motorStep(floor(angle*(STEPS_TO_FULL_ROTATION/360.0)),PLATE_RPM);
    rotatedAngles += angle;
    lcd.clear();
    lcd.print("Taking picture");
    delay(1000); //delay to let the plate come to rest
    digitalWrite(SHUTTER_PIN, HIGH);
    delay(timing);
    digitalWrite(SHUTTER_PIN, LOW);
    delay(1000); //delay to let the camera reset
  }
  lcd.clear();
}

/* scan in structured light mode
  thanks to code by *USERNAME HERE* */ //TODO: insert correct author here
void structuredLight(float angle) {

  int ch=0;
  
  while(!SELECT_FIRED) {
    lcd.println("Waiting for David");

    if(Serial.available()>0) {
      ch = Serial.read();

    }
    
    if (ch == 84){ //Wait for David to send ready signal
      ch = 0;
      
      lcd.clear();
      lcd.print("Adding Scan to list");
      Serial.println("A");            //Tell David to move the scan to the scan list

      lcd.clear();
      lcd.print("Switching to SLS");
      Serial.println("4");            //Tell David to go to structured light mode

      // FINDE HERAUS WIE WEIT GEDREHT WERDEN SOLL - LESE POTI AUS/LESE CONFIG AUS
      motorStep(floor(angle*(STEPS_TO_FULL_ROTATION/360.0)), PLATE_RPM); //rotate table

      lcd.clear();
      lcd.print("Scanning....");
      Serial.println("S");            //Tell david to scan 
    }
  }
}

void motorScan() {
  lcd.println("NOT IMPLEMENTED");
  return;
 /* int ch=0;
  
   while(true) {
    if(Serial.available()>0) {
      ch = Serial.read();
    }

    // WENN MODUS STANDALONE
    
    if (ch == 84){
      ch = 0;

      Serial.println("A");            //Tell david to move the scan to the scan list
      Serial.println("4");            //Tell David to go to structured light mode
    // FINDE HERAUS WIE WEIT GEDREHT WERDEN SOLL - LESE POTI AUS/LESE CONFIG AUS
      motorStep(15);                //rotate table 15 degrees

      Serial.println("S");            //Tell david to scan 
    }
  }*/
}

/* rotate plate 360 deg at the desired speed */
void continuosRotation(int rpm) {
  lcd.println("Rotating...");
  motorStep(STEPS_TO_FULL_ROTATION, rpm);
  lcd.println("Done");
}
