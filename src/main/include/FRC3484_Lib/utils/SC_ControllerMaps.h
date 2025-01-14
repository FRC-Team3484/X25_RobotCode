#ifndef SC_CONTROLLERMAPS_H
#define SC_CONTROLLERMAPS_H

/*=====================*/
/* XBox Controller Map */
/*=====================*/
// Analog Axes
#define XBOX_LS_X	0	// Left stick X axis
#define XBOX_LS_Y	1	// Left stick Y axis
#define XBOX_RS_X	4	// Right stick X axis
#define XBOX_RS_Y	5	// Right stick Y axis
#define XBOX_LT		2	// Left trigger
#define XBOX_RT		3	// Right trigger

// Buttons
#define XBOX_A		1
#define XBOX_B		2
#define XBOX_X		3
#define XBOX_Y		4
#define XBOX_LB		5
#define XBOX_RB		6
#define XBOX_BACK	7
#define XBOX_START	8
#define XBOX_L3		9
#define XBOX_R3		10

// D-Pad


/* DualShock 4 Controller Map */
/*============================*/
// Analog Axes
#define DS4_LS_X	0
#define DS4_LS_Y	1
#define DS4_RS_X	2
#define DS4_RS_Y	5
#define DS4_L2		3
#define DS4_R2		4

// Buttons
#define DS4_CROSS		2
#define DS4_CIRCLE		3
#define DS4_SQUARE		1
#define DS4_TRIANGLE	4
#define DS4_L1			5
#define DS4_R1			6
#define DS4_L2_BTN		7
#define DS4_R2_BTN		8
#define DS4_SHARE		9
#define DS4_OPTIONS		10
#define DS4_L3			11
#define DS4_R3			12
#define DS4_PS			13 // PlayStation button
#define DS4_TOUCH		14 // Touchpad button

/*==================================*/
/* Logitech Extreme-3D Joystick Map */
/*==================================*/
// Analog Axes
#define LE3D_X			0
#define LE3D_Y			1
#define LE3D_Z			2 // Twist axis
#define LE3D_THROTTLE	3

// Buttons
#define LE3D_TRIGGER	1
#define LE3D_BTN_2		2 // Top button (thumb rest on stick)
#define LE3D_BTN_3		3
#define LE3D_BTN_4		4
#define LE3D_BTN_5		5
#define LE3D_BTN_6		6
#define LE3D_BTN_7		7
#define LE3D_BTN_8		8
#define LE3D_BTN_9		9
#define LE3D_BTN_10		10
#define LE3D_BTN_11		11
#define LE3D_BTN_12		12

// Hat Switches
#define LE3D_HAT_Y		4	// Axis 4, reads -1/0/1 top-to-bottom
#define LE3D_HAT_X		5	// Axis 4, reads -1/0/1 left-to-right

//NOTE: The following are custom values to be used with SC_Joystick (to be written at a later time)
// #define LE3D_HAT_UP		20 // axis 4, value = -1
// #define LE3D_HAT_DOWN	21 // axis 4, value = 1
// #define LE3D_HAT_LEFT	22 // axis 5, value = -1
// #define LE3D_HAT_RIGHT	23 // axis 5, value = 1
#endif // SC_CONTROLLERMAPS_H