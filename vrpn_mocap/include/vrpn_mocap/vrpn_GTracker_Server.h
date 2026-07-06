#ifndef _VRPN_TRACKER_GTRACKER
#define _VRPN_TRACKER_GTRACKER

#include <vector>
#include "vrpn_Tracker.h"
#include "vrpn_Button.h"
#include "vrpn_Analog.h"

#define vrpn_GTRACKER_FLYSTICK_MAX_BUTTON    16  // maximum number of buttons
#define vrpn_GTRACKER_FLYSTICK_MAX_JOYSTICK  8	// maximum number of joystick values
#define VRPN_GTRACKER_FLYSTICK_BUTTON_NUM	8	// number of vrpn buttons per Flystick (fixed)
#define VRPN_GTRACKER_FLYSTICK_ANALOG_NUM	2	// number of vrpn analogs per Flystick (fixed)
#define vrpn_GTRACKER_BUFFER_LEN			1024

using std::vector;


// Data types:
typedef unsigned int uint;
// Standard marker data (3DOF):
typedef struct{
	int id;				// id number (starting with 0)

	float loc[3];			// location (in mm)
} vrpn_gtracker_marker_type;


// Standard body data (6DOF):
typedef struct{
	int id;              // id number (starting with 0)

	float loc[3];         // location (in mm)
	float qua[4];         // quaternion matrix (column-wise)
} vrpn_gtracker_body_type;


// Flystick data (6DOF + buttons):
//  - currently not tracked bodies are getting a quality of -1
typedef struct{
	int id;              // id number (starting with 0)

	float loc[3];         // location (in mm)
	float qua[4];         // quaternion matrix (column-wise)

	int num_button;       // number of buttons
	char button[vrpn_GTRACKER_FLYSTICK_MAX_BUTTON];		// button state (1 pressed, 0 not pressed)
													// (0 front, 1..n-1 right to left)
	int num_joystick;     // number of joystick
	float joystick[vrpn_GTRACKER_FLYSTICK_MAX_JOYSTICK];	// joystick value (-1 <= joystick <= 1) 
} vrpn_gtracker_flystick_type;



class vrpn_GTracker_Server:public vrpn_Tracker, public vrpn_Button, public vrpn_Analog
{

public:
	vrpn_GTracker_Server(const char *name="GTrackerServer", vrpn_Connection *m_connect=NULL);

	~vrpn_GTracker_Server();

	bool updateGTrackerBodys(uint bodyNum, int *bodyId, float (*status)[7]);

	bool updateGTrackerFlysticks(uint flystickNum, int *flystickId, float (*posStatus)[7], int *btnNum, char **btnStatus);

	virtual void mainloop();


private:
	struct timeval tim_first;
	struct timeval tim_last;

	uint m_bodyNum;
	std::vector<vrpn_gtracker_body_type> m_body;

	uint m_flystickNum;
	std::vector<vrpn_gtracker_flystick_type> m_flystick;

	vrpn_Connection *m_connect;
	
	char packetBuffer[vrpn_GTRACKER_BUFFER_LEN];
};

#endif