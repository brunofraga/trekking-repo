// Do not remove the include below
#include "Logic/trekking.h"
#include "trekking_mega.h"

#include "Logic/trekking.h"

float max_linear_vel = 1.0;
float max_angular_vel = 1.0;

Trekking trekking(max_linear_vel, max_angular_vel);
Position *cone_1 = new Position(0,0,0);
Position *cone_2 = new Position(0,0,0);
Position *cone_3 = new Position(0,0,0);

void setup() {
	trekking.start();
	trekking.addTarget(cone_1);
	trekking.addTarget(cone_2);
	trekking.addTarget(cone_3);
}

void loop() {
	trekking.update();
}
