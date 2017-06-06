#ifndef SENSORS_HW_H
#define SENSORS_HW_H
class SixForceSensor
{
	public:
		SixForceSensor()
		{
			xForce = 0;
			xTorque = 0;
			yForce = 0;
			yTorque = 0;
			zForce = 0;
			zTorque = 0;
            state = 0;
		}

		float xForce;
		float xTorque;
		float yForce;
		float yTorque;
		float zForce;
		float zTorque;
		uint32_t state;
};

class NineAxis
{
	public:
		NineAxis()
		{
			pitch = 0;
			roll = 0;
			yaw = 0;
			temp = 0;
            state = 0;
		}

		float pitch;
		float roll;
		float yaw;
		float temp;
		uint32_t state;
};

#endif
