#include "drive.hpp"

#include "EZ-Template/util.hpp"
#include "main.h"  // IWYU pragma: keep

AutonMode autonMode = AutonMode::BRAIN;
const double width = 11.5;
Coordinate currentPoint = {0, 0, 0};
vector<Coordinate> autonPath = {};

//
// Internal math
//

double getDistance(Coordinate point1, Coordinate point2, ez::drive_directions direction) {
	auto new_direction = direction == rev ? -1 : 1;
	double errorX = point2.x - point1.x;
	double errorY = point2.y - point1.y;
	return ((sqrt((errorX * errorX) + (errorY * errorY))) * new_direction);
}

double getTheta(Coordinate point1, Coordinate point2, ez::drive_directions direction) {
	auto new_direction = direction == rev ? 180 : 0;
	double errorX = point2.x - point1.x;
	double errorY = point2.y - point1.y;
	return ((atan2(errorY, errorX) * 180 / M_PI) + new_direction);
}

Coordinate getArc(Coordinate startpoint, double right, double left, double distance) {
	// Get the coordinate within the reference frame of the robot of the end point
	double radius = (right + left) / (right - left) * (width / 2);
	double theta = (right - left) / width * distance;

	double relative_x = radius * sin(theta);
	double relative_y = -radius * cos(theta) + radius;
	Coordinate point_relative = {relative_x, relative_y, theta * 180 / M_PI, (right + left) / 2, fwd};

	// Rotate the point around the origin by the start point's value of theta
	Coordinate point_global = point_relative;
	point_global.x = (point_relative.x * cos((startpoint.t + 90) * M_PI / 180)) - (point_relative.y * sin((startpoint.t + 90) * M_PI / 180));
	point_global.y = (point_relative.y * cos((startpoint.t + 90) * M_PI / 180)) + (point_relative.x * sin((startpoint.t + 90) * M_PI / 180));
	if(radius < 0) point_global.t *= -1;
	point_global.t += startpoint.t;
	if(point_global.t < 0) point_global.t += 360;
	point_global.t = fmod(point_global.t, 360);

	// Translate the point's x and y values by the start point's x and y values
	point_global.x += startpoint.x;
	point_global.y += startpoint.y;

	return point_global;
}

Coordinate getArcFromTheta(Coordinate startpoint, double right, double left, double theta) {
	// Get the coordinate within the reference frame of the robot of the end point
	double radius = (right + left) / (right - left) * (width / 2);
	theta *= right < left ? -1 : 1;
	double relative_x = radius * sin(theta * M_PI / 180);
	double relative_y = -radius * cos(theta * M_PI / 180) + radius;
	Coordinate point_relative = {relative_x, relative_y, theta, (right + left) / 2, fwd};

	// Rotate the point around the origin by the start point's value of theta
	Coordinate point_global = point_relative;
	point_global.x = (point_relative.x * cos((startpoint.t + 90) * M_PI / 180)) - (point_relative.y * sin((startpoint.t + 90) * M_PI / 180));
	point_global.y = (point_relative.y * cos((startpoint.t + 90) * M_PI / 180)) + (point_relative.x * sin((startpoint.t + 90) * M_PI / 180));
	if(radius < 0) point_global.t *= -1;
	point_global.t += startpoint.t;
	if(point_global.t < 0) point_global.t += 360;
	point_global.t = fmod(point_global.t, 360);

	// Translate the point's x and y values by the start point's x and y values
	point_global.x += startpoint.x;
	point_global.y += startpoint.y;

	return point_global;
}

std::vector<Coordinate> injectArc(Coordinate startpoint, ez::e_swing side, ez::e_angle_behavior behavior, double main, double opp, double theta,
								  double lookAhead) {
	double left = side == LEFT_SWING ? main : opp;
	double right = side == RIGHT_SWING ? main : opp;

	if(startpoint.t < 0) startpoint.t += 360;

	std::vector<Coordinate> pointsBar;
	double arciter = 0;
	double arcdist = ((side == LEFT_SWING && behavior == ez::ccw) || (side == RIGHT_SWING && behavior == ez::cw)) ? .01 * lookAhead : -.01 * lookAhead;
	Coordinate newDist = getArc(startpoint, left, right, arciter);

	if(side == LEFT_SWING) {
		theta *= -1;
		theta -= 90;
	}
	theta = arcdist > 0 ? theta + 90 : theta - 90;
	theta = fmod(theta, 360);
	if(theta < 0) theta += 360;

	while(!(newDist.t > theta - (3.837 * lookAhead) && newDist.t < theta + (3.837 * lookAhead))) {
		arciter += arcdist;
		newDist = getArc(startpoint, left, right, arciter);
		pointsBar.push_back(newDist);
	}
	return pointsBar;
}

std::vector<Coordinate> injectPath(std::vector<Coordinate> coordList, double lookAhead) {
	if(coordList.size() > 1) {
		std::vector<Coordinate> injectedList = {};
		for(int i = 0; i < coordList.size() - 1; i++) {
			if(coordList[i + 1].movement == MovementType::SWING) {
				std::vector<Coordinate> swingList = injectArc(coordList[i], coordList[i + 1].side, coordList[i + 1].behavior, coordList[i + 1].main,
															  coordList[i + 1].opp, coordList[i + 1].theta, lookAhead);
				injectedList.insert(injectedList.end(), swingList.begin(), swingList.end());
			} else if(coordList[i + 1].movement == MovementType::DRIVE) {
				ez::drive_directions dir = coordList[i].x > coordList[i + 1].x ? rev : fwd;
				double angle = getTheta(coordList[i], coordList[i + 1], dir);
				double errorX = lookAhead * (cos(angle * M_PI / 180));
				double errorY = lookAhead * (sin(angle * M_PI / 180));
				Coordinate newDist = coordList[i];
				injectedList.push_back(coordList[i]);
				while(getDistance(coordList[i], newDist, fwd) < getDistance(coordList[i], coordList[i + 1], fwd)) {
					newDist.x += errorX * (dir ? -1 : 1);
					newDist.y += errorY * (dir ? -1 : 1);
					injectedList.push_back(newDist);
				}
				injectedList.pop_back();
			}
		}
		injectedList.push_back(coordList.back());
		return injectedList;
	}
	return coordList;
}

//
// Set position wrappers
//

void setPosition(double x, double y) {
	currentPoint.x = x;
	currentPoint.y = y;
	chassis.odom_xy_set(currentPoint.x, currentPoint.y);
	currentPoint.movement = MovementType::DRIVE;
	autonPath.push_back(currentPoint);
}

void setPosition(double x, double y, double t) {
	currentPoint.x = x;
	currentPoint.y = y;
	currentPoint.t = t;
	chassis.odom_xyt_set(currentPoint.x, currentPoint.y, t);
	currentPoint.movement = MovementType::DRIVE;
	autonPath.push_back(currentPoint);
}

//
// Wait wrappers
//

void pidWait(Wait type) {
	switch(autonMode) {
		case AutonMode::PLAIN:
		case AutonMode::ODOM:
			switch(type) {
				case Wait::QUICK:
					chassis.pid_wait_quick();
					break;
				case Wait::CHAIN:
					chassis.pid_wait_quick_chain();
					break;
				case Wait::WAIT:
				default:
					chassis.pid_wait();
					break;
			}
			break;
		case AutonMode::BRAIN:
		case AutonMode::DRIVER:
		default:
			break;
	}
}

void pidWaitUntil(double distance) {
	switch(autonMode) {
		case AutonMode::PLAIN:
		case AutonMode::ODOM:
			chassis.pid_wait_until(distance * okapi::inch);
			break;
		case AutonMode::BRAIN:
		case AutonMode::DRIVER:
		default:
			break;
	}
}

void pidWaitUntil(Coordinate coordinate) {
	switch(autonMode) {
		case AutonMode::PLAIN:
		case AutonMode::ODOM:
			chassis.pid_wait_until({coordinate.x * okapi::inch, coordinate.y * okapi::inch});
			break;
		case AutonMode::BRAIN:
		case AutonMode::DRIVER:
		default:
			break;
	}
}

//
// Move to point wrappers
//

void moveToPoint(Coordinate newpoint, ez::drive_directions direction, int speed) {
	bool slew_state = false;
	switch(autonMode) {
		case AutonMode::PLAIN:
			if(getDistance({currentPoint.x, currentPoint.y}, newpoint, direction) * okapi::inch > 24_in && speed > 90) slew_state = true;
			chassis.pid_turn_set(getTheta({currentPoint.x, currentPoint.y}, newpoint, direction) * okapi::degree, speed);
			chassis.pid_wait_quick_chain();
			chassis.pid_drive_set(getDistance({currentPoint.x, currentPoint.y}, newpoint, direction) * okapi::inch, speed, slew_state);
			break;
		case AutonMode::ODOM:
			chassis.pid_odom_set({{newpoint.x * okapi::inch, newpoint.y * okapi::inch}, fwd, speed});
			break;
		case AutonMode::BRAIN:
		case AutonMode::DRIVER:
		default:
			break;
	}
	currentPoint.t = (getTheta({currentPoint.x, currentPoint.y}, newpoint, direction) * -1) + 90;
	currentPoint.x = newpoint.x;
	currentPoint.y = newpoint.y;
	currentPoint.speed = speed;
	currentPoint.facing = direction;
	currentPoint.movement = MovementType::DRIVE;
	autonPath.push_back(currentPoint);
}

void moveToPoint(Coordinate currentpoint, Coordinate newpoint, ez::drive_directions direction, int speed) {
	bool slew_state = false;
	switch(autonMode) {
		case AutonMode::PLAIN:
			if(getDistance(currentpoint, newpoint, direction) * okapi::inch > 24_in && speed > 90) slew_state = true;
			chassis.pid_turn_set(getTheta(currentpoint, newpoint, direction) * okapi::degree, speed);
			chassis.pid_wait_quick_chain();
			chassis.pid_drive_set(getDistance(currentpoint, newpoint, direction) * okapi::inch, speed, slew_state);
			break;
		case AutonMode::ODOM:
			chassis.pid_odom_set({{newpoint.x * okapi::inch, newpoint.y * okapi::inch}, fwd, speed});
			break;
		case AutonMode::BRAIN:
		case AutonMode::DRIVER:
		default:
			break;
	}
	currentPoint.x += newpoint.x - currentpoint.x;
	currentPoint.y += newpoint.y - currentpoint.y;
	currentPoint.t = (getTheta({currentPoint.x, currentPoint.y}, newpoint, direction) * -1) + 90;
	currentPoint.speed = speed;
	currentPoint.facing = direction;
	currentPoint.movement = MovementType::DRIVE;
	autonPath.push_back(currentPoint);
}

//
// Drive set wrappers
//

void driveSet(double distance, int speed, bool slew) {
	double errorX = distance * (cos(chassis.odom_theta_get() * M_PI / 180));
	double errorY = distance * (sin(chassis.odom_theta_get() * M_PI / 180));
	ez::drive_directions direction = distance < 0 ? rev : fwd;
	switch(autonMode) {
		case AutonMode::PLAIN:
			chassis.pid_drive_set(distance * okapi::inch, speed, false);
			break;
		case AutonMode::ODOM:
			chassis.pid_odom_set(distance * okapi::inch, speed, false);
			break;
		case AutonMode::BRAIN:
		case AutonMode::DRIVER:
		default:
			errorX = distance * (cos((currentPoint.movement == MovementType::TURN ? currentPoint.t - 90 : currentPoint.t) * M_PI / 180));
			errorY = distance * (sin((currentPoint.movement == MovementType::TURN ? currentPoint.t - 90 : currentPoint.t) * M_PI / 180));
			break;
	}
	currentPoint.x += errorX;
	currentPoint.y += errorY;
	currentPoint.speed = speed;
	currentPoint.facing = direction;
	currentPoint.movement = MovementType::DRIVE;
	autonPath.push_back(currentPoint);
}

void driveSet(double distance, int speed) {
	double errorX = distance * (cos(chassis.odom_theta_get() * M_PI / 180));
	double errorY = distance * (sin(chassis.odom_theta_get() * M_PI / 180));
	ez::drive_directions direction = distance < 0 ? rev : fwd;
	switch(autonMode) {
		case AutonMode::PLAIN:
			chassis.pid_drive_set(distance * okapi::inch, speed, false);
			break;
		case AutonMode::ODOM:
			chassis.pid_odom_set(distance * okapi::inch, speed, false);
			break;
		case AutonMode::BRAIN:
		case AutonMode::DRIVER:
		default:
			errorX = distance * (cos((currentPoint.movement == MovementType::TURN ? currentPoint.t - 90 : currentPoint.t) * M_PI / 180));
			errorY = distance * (sin((currentPoint.movement == MovementType::TURN ? currentPoint.t - 90 : currentPoint.t) * M_PI / 180));
			break;
	}
	currentPoint.x += errorX;
	currentPoint.y += errorY;
	currentPoint.speed = speed;
	currentPoint.facing = direction;
	currentPoint.movement = MovementType::DRIVE;
	autonPath.push_back(currentPoint);
}

//
// Turn set wrappers
//

void turnSet(double theta, int speed) {
	switch(autonMode) {
		case AutonMode::PLAIN:
		case AutonMode::ODOM:
			chassis.pid_turn_set(theta * okapi::degree, speed);
			break;
		case AutonMode::BRAIN:
		case AutonMode::DRIVER:
		default:
			break;
	}
	currentPoint.movement = MovementType::TURN;
	currentPoint.t = theta;
	autonPath.push_back(currentPoint);
}

//
// Swing set wrappers
//

void swingSet(ez::e_swing side, double theta, double main, double opp, ez::e_angle_behavior behavior) {
	switch(autonMode) {
		case AutonMode::PLAIN:
		case AutonMode::ODOM:
			chassis.pid_swing_set(side, theta * okapi::degree, main, opp, behavior);
			break;
		case AutonMode::BRAIN:
		case AutonMode::DRIVER:
		default:
			break;
	}
	double right = side == RIGHT_SWING ? main : opp;
	double left = side == LEFT_SWING ? main : opp;
	bool swingtype = ((side == LEFT_SWING && behavior == ez::ccw) || (side == RIGHT_SWING && behavior == ez::cw)) ? true : false;
	if(side == LEFT_SWING) {
		theta *= -1;
		theta -= 180;
	}
	if(!swingtype && currentPoint.t < 0) theta -= 180;
	theta = fmod(theta, 360);
	currentPoint = getArcFromTheta(currentPoint, left, right, swingtype ? theta + 90 : theta - 90);
	if(!swingtype && currentPoint.t > 0) theta -= 180;
	currentPoint.t = swingtype ? 90 - theta : 270 - theta;
	currentPoint.movement = MovementType::SWING;
	currentPoint.side = side;
	currentPoint.theta = swingtype ? theta - 90 : theta + 270;
	currentPoint.main = main;
	currentPoint.opp = opp;
	currentPoint.behavior = behavior;
	autonPath.push_back(currentPoint);
}

void swingSet(ez::e_swing side, double theta, double main, ez::e_angle_behavior behavior) {
	switch(autonMode) {
		case AutonMode::PLAIN:
		case AutonMode::ODOM:
			chassis.pid_swing_set(side, theta * okapi::degree, main, 0, behavior);
			break;
		case AutonMode::BRAIN:
		case AutonMode::DRIVER:
		default:
			break;
	}
	double right = side == RIGHT_SWING ? main : 0;
	double left = side == LEFT_SWING ? main : 0;
	bool swingtype = ((side == LEFT_SWING && behavior == ez::ccw) || (side == RIGHT_SWING && behavior == ez::cw)) ? true : false;
	if(side == LEFT_SWING) {
		theta *= -1;
		theta -= 180;
	}
	if(!swingtype && currentPoint.t < 0) theta -= 180;
	theta = fmod(theta, 360);
	currentPoint = getArcFromTheta(currentPoint, left, right, swingtype ? theta + 90 : theta - 90);
	if(!swingtype && currentPoint.t > 0) theta -= 180;
	currentPoint.t = swingtype ? 90 - theta : 270 - theta;
	currentPoint.movement = MovementType::SWING;
	currentPoint.side = side;
	currentPoint.theta = swingtype ? theta - 90 : theta + 270;
	currentPoint.main = main;
	currentPoint.opp = 0;
	currentPoint.behavior = behavior;
	autonPath.push_back(currentPoint);
}

void swingSet(ez::e_swing side, double theta, double main, double opp) {
	e_angle_behavior behavior = ((theta > 180) && (side == RIGHT_SWING)) || ((theta < 180) && (side == LEFT_SWING)) ? cw : ccw;
	switch(autonMode) {
		case AutonMode::PLAIN:
		case AutonMode::ODOM:
			chassis.pid_swing_set(side, theta * okapi::degree, main, opp, behavior);
			break;
		case AutonMode::BRAIN:
		case AutonMode::DRIVER:
		default:
			break;
	}
	double right = side == RIGHT_SWING ? main : opp;
	double left = side == LEFT_SWING ? main : opp;
	bool swingtype = ((side == LEFT_SWING && behavior == ez::ccw) || (side == RIGHT_SWING && behavior == ez::cw)) ? true : false;
	if(side == LEFT_SWING) {
		theta *= -1;
		theta -= 180;
	}
	if(!swingtype && currentPoint.t < 0) theta -= 180;
	theta = fmod(theta, 360);
	currentPoint = getArcFromTheta(currentPoint, left, right, swingtype ? theta + 90 : theta - 90);
	if(!swingtype && currentPoint.t > 0) theta -= 180;
	currentPoint.t = swingtype ? 90 - theta : 270 - theta;
	currentPoint.movement = MovementType::SWING;
	currentPoint.side = side;
	currentPoint.theta = swingtype ? theta - 90 : theta + 270;
	currentPoint.main = main;
	currentPoint.opp = opp;
	currentPoint.behavior = behavior;
	autonPath.push_back(currentPoint);
}

void swingSet(ez::e_swing side, double theta, double main) {
	e_angle_behavior behavior = ((theta > 180) && (side == RIGHT_SWING)) || ((theta < 180) && (side == LEFT_SWING)) ? cw : ccw;
	switch(autonMode) {
		case AutonMode::PLAIN:
		case AutonMode::ODOM:
			chassis.pid_swing_set(side, theta, main);
			break;
		case AutonMode::BRAIN:
		case AutonMode::DRIVER:
		default:
			break;
	}
	double right = side == RIGHT_SWING ? main : 0;
	double left = side == LEFT_SWING ? main : 0;
	bool swingtype = ((side == LEFT_SWING && behavior == ez::ccw) || (side == RIGHT_SWING && behavior == ez::cw)) ? true : false;
	if(side == LEFT_SWING) {
		theta *= -1;
		theta -= 180;
	}
	if(!swingtype && currentPoint.t < 0) theta -= 180;
	theta = fmod(theta, 360);
	currentPoint = getArcFromTheta(currentPoint, left, right, swingtype ? theta + 90 : theta - 90);
	if(!swingtype && currentPoint.t > 0) theta -= 180;
	currentPoint.t = swingtype ? 90 - theta : 270 - theta;
	currentPoint.movement = MovementType::SWING;
	currentPoint.side = side;
	currentPoint.theta = swingtype ? theta - 90 : theta + 270;
	currentPoint.main = main;
	currentPoint.opp = 0;
	currentPoint.behavior = behavior;
	autonPath.push_back(currentPoint);
}

//
// Print path
//

void getPath() {
	cout << "===========================================" << endl;
	for(auto point : autonPath) {
		cout << "(" << point.x << ", " << point.y << ")" << endl;
	}
	cout << "===========================================" << endl;
}

void getPathInjected() {
	auto injected = injectPath(autonPath, 2);
	cout << "===========================================" << endl;
	for(auto point : injected) {
		cout << "(" << point.x << ", " << point.y << ")" << endl;
	}
	cout << "===========================================" << endl;
}