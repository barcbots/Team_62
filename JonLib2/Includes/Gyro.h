#pragma systemFile
#warning "JonLib2: Gyro"

int gyroCurve (int target);

typedef struct {
	pid controller;
	tSensors sensor;
	int maxSpeed;
} gyroscope;

void initPIDGyroscope (gyroscope *gyroController, tSensors sensor, float kP,  float kI, float kD, int threshold = 10, int integralLimit = -1, int slewRate = 10) {
	pid *controller = gyroController->controller;
	initPIDController(controller, kP, kI, kD, threshold, integralLimit, slewRate);
	gyroController->sensor = sensor;
}

bool leftSwingTurnGyroPID (gyroscope *gyroController) {
	pid *controller = gyroController->controller;

	long lastUpdate = nPgmTime;
	int lastError = -1;

	writeDebugStream("INTENDED: (%d) ", controller->target);

	setRightWheelSpeed(0);

	do {
		setLeftWheelSpeed(limit(-updatePIDController(controller, gyroController->sensor), gyroController->maxSpeed));

		if(abs(controller->error)<=(abs(lastError)-5))
			lastUpdate = nPgmTime;

		lastError = controller->error;

		if((nPgmTime-lastUpdate)>MOVE_TIMEOUT) {
			setWheelSpeed(0);
			writeDebugStream("TIMEOUT ");
			writeDebugStreamLine("ACT: (%d)", SensorValue[gyroController->sensor]);
			return false;
		}

		if(abs(controller->error)>controller->threshold)
			clearTimer(T4);

		delay(25);

	} while(time1[T4]<100);

	setWheelSpeed(0);

	writeDebugStreamLine("ACT: (%d)", SensorValue[gyroController->sensor]);
	return true;
}

bool rightSwingTurnGyroPID (gyroscope *gyroController) {
pid *controller = gyroController->controller;

	long lastUpdate = nPgmTime;
	int lastError = -1;

	writeDebugStream("INTENDED: (%d) ", controller->target);

	setLeftWheelSpeed(0);

	do {
		setRightWheelSpeed(limit(updatePIDController(controller, gyroController->sensor), gyroController->maxSpeed));

		if(abs(controller->error)<=(abs(lastError)-5))
			lastUpdate = nPgmTime;

		lastError = controller->error;

		if((nPgmTime-lastUpdate)>MOVE_TIMEOUT) {
			setWheelSpeed(0);
			writeDebugStreamLine("TIMEOUT");
			writeDebugStreamLine("ACT: (%d)", SensorValue[gyroController->sensor]);
			return false;
		}

		if(abs(controller->error)>controller->threshold)
			clearTimer(T4);

		delay(25);

	} while(time1[T4]<100);

	setWheelSpeed(0);

	writeDebugStreamLine("ACT: (%d)", SensorValue[gyroController->sensor]);
	return true;
}

bool pointTurnGyroPID (gyroscope *gyroController) {
	pid *controller = gyroController->controller;

	long lastUpdate = nPgmTime;
	int lastError = -1;

	writeDebugStream("INTENDED: (%d) ", controller->target);

	do {
		spin(limit(updatePIDController(controller, gyroController->sensor), gyroController->maxSpeed));

		if(abs(controller->error)<=(abs(lastError)-5))
			lastUpdate = nPgmTime;

		lastError = controller->error;

		if((nPgmTime-lastUpdate)>MOVE_TIMEOUT) {
			setWheelSpeed(0);
			writeDebugStreamLine("TIMEOUT");
			writeDebugStreamLine("ACT: (%d)", SensorValue[gyroController->sensor]);
			return false;
		}

		if(abs(controller->error)>controller->threshold)
			clearTimer(T4);

		delay(25);

	} while(time1[T4]<100);

	setWheelSpeed(0);

	writeDebugStreamLine("ACT: (%d)", SensorValue[gyroController->sensor]);
	return true;
}

void setGyroTargetPID (gyroscope *gyroController, float target, int maxSpeed = 127) {
	gyroController->maxSpeed = maxSpeed;
	pid *controller = gyroController->controller;
	clearIntegral(controller);
	controller->target = gyroCurve(target);
}

void setGyroTargetPIDAutoPointTurn (gyroscope *gyroController, float target, int maxSpeed = 127) {
	setGyroTargetPID(gyroController, target, maxSpeed);
	pointTurnGyroPID(gyroController);
}

void setGyroTargetPIDAutoRightSwingTurn (gyroscope *gyroController, float target, int maxSpeed = 127) {
	setGyroTargetPID(gyroController, target, maxSpeed);
	rightSwingTurnGyroPID(gyroController);
}

void setGyroTargetPIDAutoLeftSwingTurn (gyroscope *gyroController, float target, int maxSpeed = 127) {
	setGyroTargetPID(gyroController, target, maxSpeed);
	leftSwingTurnGyroPID(gyroController);
}

void addGyroTargetPID (gyroscope *gyroController, float target, int maxSpeed = 127) {
	gyroController->maxSpeed = maxSpeed;
	pid *controller = gyroController->controller;
	clearIntegral(controller);
	controller->target = controller->target + gyroCurve(target);
}

void addGyroTargetPIDAutoPointTurn (gyroscope *gyroController, float target, int maxSpeed = 127) {
	addGyroTargetPID(gyroController, target, maxSpeed);
	pointTurnGyroPID(gyroController);
}

void addGyroTargetPIDAutoRightSwingTurn (gyroscope *gyroController, float target, int maxSpeed = 127) {
	addGyroTargetPID(gyroController, target, maxSpeed);
	rightSwingTurnGyroPID(gyroController);
}

void addGyroTargetPIDAutoLeftSwingTurn (gyroscope *gyroController, float target, int maxSpeed = 127) {
	addGyroTargetPID(gyroController, target, maxSpeed);
	leftSwingTurnGyroPID(gyroController);
}
