#include <Joystick.h>

#define CORAL_LEVEL_4_LEFT 13
#define CORAL_LEVEL_4_RIGHT 12
#define CORAL_LEVEL_3_LEFT 14
#define CORAL_LEVEL_3_RIGHT 11
#define CORAL_LEVEL_2_LEFT 15
#define CORAL_LEVEL_2_RIGHT 10
#define CORAL_LEVEL_1 2

#define ALGAE_LEVEL_3 8
#define ALGAE_LEVEL_2 18

#define GROUND_ALGAE 16
#define GROUND_CORAL 9
#define PROCESSOR 7
#define CLIMB_UP 19
#define CLIMB_DOWN 6
#define NET 17
#define IGNORE_VISION 5
#define LOAD_CORAL 16

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_GAMEPAD,
  14, 0,
  true, true, false,
  false, false, false,
  false, false,
  false, false, false);


void setup()  {

 pinMode(CORAL_LEVEL_4_LEFT, INPUT_PULLUP); // INTAKE BOTH IN / CUBE MID
 pinMode(CORAL_LEVEL_4_RIGHT, INPUT_PULLUP); // INTAKE BOTH OUT / CONE MID
 pinMode(CORAL_LEVEL_3_LEFT, INPUT_PULLUP); // CLAW GRIP / UNIVERSAL SCORE
 pinMode(CORAL_LEVEL_3_RIGHT, INPUT_PULLUP); //           / HOME
 pinMode(CORAL_LEVEL_2_LEFT, INPUT_PULLUP); // INTAKE LEFT IN / CUBE HIGH
 pinMode(CORAL_LEVEL_2_RIGHT, INPUT_PULLUP); // INTAKE RIGHT IN / CONE HIGH
 pinMode(CORAL_LEVEL_1, INPUT_PULLUP); // CLAW TILT / FEEDER
 pinMode(ALGAE_LEVEL_3, INPUT_PULLUP); // BUTTON TOGGLE
 pinMode(ALGAE_LEVEL_2, INPUT_PULLUP); // ELEVATOR UP/DOWN
 pinMode(GROUND_ALGAE, INPUT_PULLUP); // ELEVATOR ANGLE
 pinMode(GROUND_CORAL, INPUT_PULLUP);
 pinMode(PROCESSOR, INPUT_PULLUP); // ELEVATOR ANGLE
 pinMode(CLIMB_UP, INPUT_PULLUP); // ELEVATOR ANGLE
 pinMode(CLIMB_DOWN, INPUT_PULLUP); // ELEVATOR ANGLE
 pinMode(NET, INPUT_PULLUP); // ELEVATOR ANGLE
 pinMode(IGNORE_VISION, INPUT_PULLUP); // ELEVATOR ANGLE
 pinMode(LOAD_CORAL, INPUT_PULLUP); // ELEVATOR ANGLE
  

 Joystick.begin();
 Joystick.setXAxisRange(-100, 100);
 Joystick.setYAxisRange(-100, 100);
}

void loop() {
  {
  //  int HI_input = analogRead(ELEV_HEIGHT);
  // //  double HI_VAL = ((HI_input-512.0)/512.0);
  // //  Joystick.setYAxis(HI_VAL);
  //  Joystick.setYAxis(map(HI_input, 0, 1023, -100, 100));
   

  //  int AN_input = analogRead(ELEV_ANGLE);
  // //  double AN_VAL = ((AN_input-512.0)/512.0);
  // //  Joystick.setXAxis(AN_VAL);
  //  Joystick.setXAxis(map(AN_input, 0, 1023, -100, 100));

   Joystick.setButton(0, !digitalRead(CORAL_LEVEL_4_LEFT));    // 1
   Joystick.setButton(1, !digitalRead(CORAL_LEVEL_4_RIGHT));   // 2
   Joystick.setButton(2, !digitalRead(CORAL_LEVEL_3_LEFT));    // 3
   Joystick.setButton(3, !digitalRead(CORAL_LEVEL_3_RIGHT) );                // 4

   Joystick.setButton(4, !digitalRead(CORAL_LEVEL_2_LEFT));     // 5
   Joystick.setButton(5, !digitalRead(CORAL_LEVEL_2_RIGHT));     // 6
   Joystick.setButton(6, !digitalRead(CORAL_LEVEL_1));       // 7


   Joystick.setButton(7, !digitalRead(ALGAE_LEVEL_3) );  // 8
   Joystick.setButton(8, !digitalRead(ALGAE_LEVEL_2) ); // 9
   Joystick.setButton(9, !digitalRead(GROUND_ALGAE) );  // 10
   Joystick.setButton(10, !digitalRead(GROUND_CORAL) );               // 11

   Joystick.setButton(11, !digitalRead(PROCESSOR) );   // 12
   Joystick.setButton(12, !digitalRead(CLIMB_UP) );   // 13
   Joystick.setButton(13, !digitalRead(CLIMB_DOWN) );     // 14
   Joystick.setButton(13, !digitalRead(NET) );
   Joystick.setButton(13, !digitalRead(IGNORE_VISION) );
   Joystick.setButton(13, !digitalRead(LOAD_CORAL) );
//     #define GROUND_ALGAE = 16;
// #define GROUND_CORAL = 9;
// #define PROCESSOR = 7;
// #define CLIMB_UP = 19;
// #define CLIMB_DOWN = 6;
// #define NET = 17;
// #define IGNORE_VISION = 5;
// #define LOAD_CORAL = 16;
   //Joystick.setButton(8, !digitalRead(B_INTAKEIN_CUBEMID)&&!digitalRead(TOGGLE)); //8

   //Joystick.setButton(7, !digitalRead(B_INTAKEOUT_CONEMID)&&!digitalRead(TOGGLE));  //8
   //Joystick.setButton(8, !digitalRead(CLAWGRIP_UNIVERSAL)&&!digitalRead(TOGGLE)); //9
   //Joystick.setButton(9, !digitalRead(HOME)&&!digitalRead(TOGGLE));               //10
   //Joystick.setButton(10, !digitalRead(L_INTAKEIN_CUBEHI)&&!digitalRead(TOGGLE)); //11
   //Joystick.setButton(11, !digitalRead(R_INTAKEIN_CONEHI)&&!digitalRead(TOGGLE)); //12
   //Joystick.setButton(12, !digitalRead(CLAWTILT_FEEDER)&&!digitalRead(TOGGLE));   //13
   

  }
 delay(10);
}