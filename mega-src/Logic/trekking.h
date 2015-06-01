#ifndef TREKKING_H
#define TREKKING_H

#include "../MinervaBots/Robot/robot.h"
#include "../General/Log/log.h"
#include "../General/Position/position.h"
#include "../General/Timer/timer.h"

#include "../Sensors/XLMaxSonarEZ/sonarlist.h"
#include "../trekkingpins.h"

#include "locator.h"
#include "trekkingmath.h"


// enum OperationMode {

// 	STANDBY,
// 	SEARCH_STAGE,
// 	REFINED_SEARCH_STAGE,
// 	LIGHTING
// };

class Trekking {
private:

	float linear_velocity;
	float angular_velocidy;

	LinkedList<Position *> targets;
	Position trekking_position;
	int current_target_index;

	const int COMMAND_BAUD_RATE;
	const int LOG_BAUD_RATE;
	const int ENCODER_BAUD_RATE;

	const byte MAX_MOTOR_PWM;
	const int LIGHT_DURATION;

	/*
		Input states
	*/

	bool init_button;
	bool emergency_button;
	bool operation_mode_switch;

	int min_distance_to_enable_lights;
	int min_distance_to_refine_search;

	bool is_aligned;

	char current_command;

	bool sirene_is_on;

	Robot robot;
	
	//Sonars
	SonarList sonar_list;
	XLMaxSonarEZ right_sonar;
	XLMaxSonarEZ left_sonar;
	XLMaxSonarEZ center_sonar;

	Log log;
	// Kalman kalman;
	// Radio radio;

	//Holds witch is the serial stream to receive
	//the commands
	Stream *command_stream;
	Stream *log_stream;
	Stream *encoder_stream;

	Locator locator;

	//Timers
	TimerForMethods<Locator> encoders_timer;
	TimerForMethods<Trekking> sirene_timer;

	void reset();
	void stop();
	void readInputs();
	void turnOnSirene();
	void turnOffSirene();

	void startTimers();
	void stopTimers();
	void resetTimers();
	void updateTimers();

	//Returns 1 if all the sensors are working
	bool checkSensors();

	/*----Operation modes----*/
	void standby();
	void search();
	void refinedSearch();
	void lighting();
	void (Trekking::*operation_mode)(void);

	void goToNextTarget();

	void debug();

public:
	Trekking();
	~Trekking();

	void addTarget(Position *target);

	void update();

	void start();
};

#endif //TREKKING_H
