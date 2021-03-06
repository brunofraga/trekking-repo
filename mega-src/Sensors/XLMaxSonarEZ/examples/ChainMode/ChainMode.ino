/*
	This is an example of XLMaxSonarEZ. More information at 
	https://github.com/brunocalou/XLMaxSonarEZ

	Created by Bruno Calou Alves, March, 2015 - brunocaloualves@gmail.com
*/

#include "sonarlist.h"

#define RX 9	//Trigger pin
#define TX A0 	//Read pin

#define TX_2 A1
#define TX_3 A2

//Create objects
XLMaxSonarEZ sonar1(TX, RX);
XLMaxSonarEZ sonar2(TX_2);
XLMaxSonarEZ sonar3(TX_3);

//Create the list on the Chain mode. Note that
//the Chain Loop and the Simultaneous works
//with the same code
SonarList sonar_list(Sonar::CHAIN);
//SonarList sonar_list(Sonar::CHAIN_LOOP);
//SonarList sonar_list(Sonar::SIMULTANEOUS);

void setup() {

	Serial.begin(9600);

	//Add all the sonars to the list. Note that
	//the order that the sensors are added is
	//not important, as long as the addFirst
	//method is used.
	sonar_list.addFirst(&sonar1);
	sonar_list.addSonar(&sonar2);
	sonar_list.addSonar(&sonar3);
}

void loop(){
	//Read all the sensors
	sonar_list.read();

	//Print all the values
	Serial.print(sonar1.getDistance());
	Serial.print(" - ");
	Serial.print(sonar2.getDistance());
	Serial.print(" - ");
	Serial.println(sonar3.getDistance());

	//Delay just to give us time to read the data on the serial monitor
	delay(300);
}
