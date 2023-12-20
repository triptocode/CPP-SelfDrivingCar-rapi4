#include <pigpio.h>
#include <iostream>
#include <csignal>
#include <pthread.h>
#include <thread>
#include <chrono>
#include "motor_control.h"
#include <unistd.h>
#include <cmath>

bool on_flag = false;
bool running = true;

double x = 0.0;
double y = 10.0;
double z = 20.0;

const double OBSTACLE_DISTANCE_THRESHOLD = 5.0;   // Distance threshold to detect an obstacle
const double REVERSE_DISTANCE_THRESHOLD = 10.0;  // Distance threshold to initiate reverse

const char* device = "/dev/ttyS0";
const int baudrate = 9600;

char* mutableDevice = const_cast<char*>(device);
int serial_port = serOpen(mutableDevice, baudrate, 0);

class UltraSound {
public:
    void Init(unsigned int trigger, unsigned int echo, int sensorNumber, double xCoordinate);
    double GetDistance(unsigned int timeout);
    double GetHorizontalDistance(const UltraSound& otherSensor) const;
    int GetSensorNumber() const;
    double GetLatestDistance() const;
    void SetLatestDistance(double distance);

private:
    void RecordPulseLength();
    int trigger;
    int echo;
    int sensorNumber;
    double xCoordinate;

    volatile long startTimeUsec;
    volatile long endTimeUsec;

    double distanceCm;
    double previousDistanceCm;
    long DifferenceTimeUsec;
    long now;

    pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

    static const double DISTANCE_CONVERSION_FACTOR;
};

void *ThreadFunction(void* arg);

void AdjustSpeedBasedOnDistance(double distance, int& speed);

const double UltraSound::DISTANCE_CONVERSION_FACTOR = 58.7734;

void UltraSound::Init(unsigned int trigger, unsigned int echo, int sensorNumber, double xCoordinate) {
    this->trigger = trigger;
    this->echo = echo;
    this->sensorNumber = sensorNumber;
    this->xCoordinate = xCoordinate;

    gpioSetMode(trigger, PI_OUTPUT);
    gpioSetMode(echo, PI_INPUT);
    gpioWrite(trigger, 0);
    gpioSleep(PI_TIME_RELATIVE, 0.5, 0);

    if (pthread_mutex_init(&mutex, nullptr) != 0) {
        std::cerr << "Mutex initialization failed. Exiting..." << std::endl;
        exit(-1);
    }
}
double UltraSound::GetHorizontalDistance(const UltraSound& otherSensor) const {
    return std::abs(xCoordinate - otherSensor.xCoordinate);
}

double UltraSound::GetDistance(unsigned int timeout) {
    pthread_mutex_lock(const_cast<pthread_mutex_t*>(&mutex));

    endTimeUsec = 0;
    gpioWrite(trigger, 1);
    gpioDelay(10);
    gpioWrite(trigger, 0);

    now = gpioTick();

    while (endTimeUsec == 0 && gpioTick() - now < timeout) {
        RecordPulseLength();
    }

    DifferenceTimeUsec = endTimeUsec - startTimeUsec;
    distanceCm = (double)DifferenceTimeUsec / DISTANCE_CONVERSION_FACTOR;

    if (distanceCm == previousDistanceCm) {
        pthread_mutex_unlock(const_cast<pthread_mutex_t*>(&mutex));
        return distanceCm;
    }

    previousDistanceCm = distanceCm;

    pthread_mutex_unlock(const_cast<pthread_mutex_t*>(&mutex));

    if (endTimeUsec != 0)
        return distanceCm;
    else
        return 0;
}

double UltraSound::GetLatestDistance() const {
    pthread_mutex_lock(const_cast<pthread_mutex_t*>(&mutex));

    double latestDistance = distanceCm;

    pthread_mutex_unlock(const_cast<pthread_mutex_t*>(&mutex));

    return latestDistance;
}

void UltraSound::SetLatestDistance(double distance) {
    pthread_mutex_lock(const_cast<pthread_mutex_t*>(&mutex));

    distanceCm = distance;

    pthread_mutex_unlock(const_cast<pthread_mutex_t*>(&mutex));
}

void UltraSound::RecordPulseLength() {
    startTimeUsec = gpioTick();
    while (gpioRead(echo) == 1) {
        endTimeUsec = gpioTick();
    }
}

int UltraSound::GetSensorNumber() const {
    return sensorNumber;
}

int calculateSpeed(double distance) {
    if (distance < 10.0) {
        return 128;
    } else if (distance < 20.0) {
        return 128;
    } else {
        return 128;
    }
}

double CalculateAngle(double distance1, double distance3) {
    double horizontalDistance = 20.0;

    double tangentValue = (distance1 - distance3) / horizontalDistance;
    double angleInDegrees = atan(tangentValue) * (180.0 / M_PI);

    return angleInDegrees;
}

const double TURN_THRESHOLD_DISTANCE = 38.0;
const double TURN_ANGLE_THRESHOLD = 45.0;

void *ThreadFunction(void* arg) {
    UltraSound* us1 = new UltraSound();
    us1->Init(0, 1, 1, x);

    UltraSound* us2 = new UltraSound();
    us2->Init(5, 6, 2, y);

    UltraSound* us3 = new UltraSound();
    us3->Init(26, 16, 3, z);

    while (running) {
        gpioSleep(PI_TIME_RELATIVE, 0, 500000);
        double distance1 = us1->GetDistance(30000);
        double distance2 = us2->GetDistance(30000);
        double distance3 = us3->GetDistance(30000);

        us1->SetLatestDistance(distance1);
        us2->SetLatestDistance(distance2);
        us3->SetLatestDistance(distance3);

        std::cout << "U1 Distance: " << distance1 << " cm." << std::endl;
        std::cout << "U2 Distance: " << distance2 << " cm." << std::endl;
        std::cout << "U3 Distance: " << distance3 << " cm." << std::endl;

        double avgDistance = (distance1 + distance2 + distance3) / 3.0;
        int speed = calculateSpeed(avgDistance);

        AdjustSpeedBasedOnDistance(avgDistance, speed);

        double angle = CalculateAngle(distance1, distance3);

        std::cout << "Angle: " << angle << " degrees." << std::endl;
        std::cout << "Distance: " << avgDistance << " cm." << std::endl;

        if (on_flag) {
            std::cout << "Continuing forward" << std::endl;
            Driving_forward(IN1, IN2, IN3, IN4, 113);

            // Check distance and perform left or right turn logic
	    if( distance2 <= TURN_THRESHOLD_DISTANCE+3) //-5.0
	     {

		if(distance1==distance3){

			Driving_forward(IN1,IN2,IN3,IN4,110);

			//std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
		if(distance1 >= distance3)
		{
			Driving_left(IN1, IN2, IN3, IN4, 111);
		//	gpioPWM(ENABLE, 128);
		//	gpioPWM(ENB, 128);	

			/*if(distance1<TURN_THRESHOLD_DISTANCE)
			{
				Driving_right(IN1,IN2,IN3,IN4,100);
			)*/
		}
		else if (distance3 >= distance1)
		{
			Driving_right(IN1, IN2, IN3, IN4, 111);

			//Driving_forward(IN1,IN2,IN3,IN4,130);
		//	gpioPWM(ENABLE, 128);
		//	gpioPWM(ENB, 128);

			/*if(distance3<TURN_THRESHOLD_DISTANCE)
			{
				Driving_left(IN1,IN2,IN3,IN4,100);
			//	gpioPWM(ENABLE,128);
			//	gpioPWM(ENB,128);
			}*/
		}
	//	else{
}	
	  //   }
            if (distance1<=TURN_THRESHOLD_DISTANCE+10&&distance2<=TURN_THRESHOLD_DISTANCE+10&&distance3<=TURN_THRESHOLD_DISTANCE+10) {
                // If all sensors detect an obstacle, stop
                //std::cout << "Stopping due to obstacles" << std::endl;
                Driving_backward(IN1, IN2, IN3, IN4,120);
               // gpioPWM(ENABLE, 128);
               // gpioPWM(ENB, 128);
//		std::this_thread::sleep_for(std::chrono::seconds(1));

		Driving_right(IN1,IN2,IN3,IN4,110);

		//Driving_forward(IN1,IN2,IN3,IN4,130);
                //Driving_backward(IN1, IN2, IN3, IN4, 70);
               // gpioPWM(ENABLE, 128);
               // gpioPWM(ENB, 128);
  //              std::this_thread::sleep_for(std::chrono::seconds(2));


               /* if(distance1>distance3)
                {


                    Driving_left(IN1, IN2, IN3, IN4, 100);
                    gpioPWM(ENABLE, 128);
                    gpioPWM(ENB, 128);


                }

                if(distance3>distance1)
                {


                    Driving_right(IN1, IN2, IN3, IN4, 100);
                    gpioPWM(ENABLE, 128);
	 	    gpioPWM(ENB, 128);
		}*/
                //std::this_thread::sleep_for(std::chrono::seconds(1));
            } else if (distance1 <= TURN_THRESHOLD_DISTANCE-5.0 && distance2 >=45.0&&distance3>TURN_THRESHOLD_DISTANCE-3.0) {
                // If left sensor detects an obstacle, perform right turn
                std::cout << "Performing right turn" << std::endl;
                Driving_right(IN1, IN2, IN3, IN4, 105);

		//Driving_forward(IN1,IN2,IN3,IN4,130);
               // gpioPWM(ENABLE, 128);
               // gpioPWM(ENB, 128);
  //              std::this_thread::sleep_for(std::chrono::seconds(1));
            } else if (distance1>TURN_THRESHOLD_DISTANCE-5.0&&distance3 <= TURN_THRESHOLD_DISTANCE-5 && distance2 >= 45.0) {
                // If right sensor detects an obstacle, perform left turn
                std::cout << "Performing left turn" << std::endl;
                Driving_left(IN1, IN2, IN3, IN4, 115);
               // gpioPWM(ENABLE, 128);
               // gpioPWM(ENB, 128);
//                std::this_thread::sleep_for(std::chrono::seconds(1));
            }// else if(distance3<TURN_THRESHOLD_DISTANCE)
//		{
		//	Driving_left(IN1,IN2,IN3,IN4,128);


	//	}else if(distance1<TURN_THRESHOLD_DISTANCE)
//		{
		//	Driving_right(IN1,IN2,IN3,IN4,128);
		//}
        }
    }

    delete us1;
    delete us2;
    delete us3;

    return nullptr;
}
void AdjustSpeedBasedOnDistance(double distance, int& speed) {
    // Adjust speed based on distance if needed
}



/*void ReverseForDuration() {
    Driving_backward(IN1, IN2, IN3, IN4, 70);
   // gpioPWM(ENABLE, 128);
   // gpioPWM(ENB, 128);

    std::this_thread::sleep_for(std::chrono::seconds(1));

    Driving_stop(IN1, IN2, IN3, IN4, 100);
   // gpioPWM(ENABLE, 0);
   // gpioPWM(ENB, 0);
}*/

void controlMotors(char input, UltraSound& us1, UltraSound& us2, UltraSound& us3) {
    double avgDistance = (us1.GetLatestDistance() + us2.GetLatestDistance() + us3.GetLatestDistance()) / 3.0;
    int speed = calculateSpeed(avgDistance);

    if (avgDistance < OBSTACLE_DISTANCE_THRESHOLD) {
        std::cout << "Obstacle detected, reversing for 1 second." << std::endl;
        // ReverseForDuration();
        // return;
    }

    if (input == '2') {
        std::cout << "Stopping motors" << std::endl;
        Driving_stop(IN1, IN2, IN3, IN4, 100);
       // gpioPWM(ENABLE, 0);
       // gpioPWM(ENB, 0);
        return;
    }
}

UltraSound US1, US2, US3;

void *BluetoothControlThread(void* arg) {
    int serial_port = *(int*)arg;

    try {
        while (running) {
            if (serDataAvailable(serial_port) > 0) {
                char input = serReadByte(serial_port);
                if (input == '2' && on_flag) {
                    std::cout << "Stopping motors" << std::endl;
                    Driving_stop(IN1, IN2, IN3, IN4, 100);
                    //gpioPWM(ENABLE, 0);
                    //gpioPWM(ENB, 0);
                    on_flag = false;
                } else if (input == '1') {
                    on_flag = true;
                }
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Bluetooth communication error: " << e.what() << std::endl;
    }

    return nullptr;
}

void signalHandler(int signum) {
    gpioPWM(ENABLE, 0);
    gpioPWM(ENB, 0);
    gpioTerminate();
    exit(signum);
}
int main() {
    gpioInitialise();

    signal(SIGINT, signalHandler);

    const char* device = "/dev/ttyS0";
    const int baudrate = 9600;

    if (gpioInitialise() < 0) {
        std::cerr << "GPIO initialization failed. Exiting..." << std::endl;
        return 1;
    }

    char* mutableDevice = const_cast<char*>(device);
    int serial_port = serOpen(mutableDevice, baudrate, 0);
    if (serial_port < 0) {
        std::cerr << "Failed to open serial port. Exiting..." << std::endl;
        gpioTerminate();
        return 1;
    }

    double xCoordinateUS1 = 0.0;
    double xCoordinateUS2 = 10.0;
    double xCoordinateUS3 = 20.0;

    US1.Init(0, 1, 1, xCoordinateUS1);
    US2.Init(5, 6, 2, xCoordinateUS2);
    US3.Init(26, 16, 3, xCoordinateUS3);

    pthread_t thread1, thread2, thread3, bluetoothThread;

    if (pthread_create(&thread1, nullptr, ThreadFunction, reinterpret_cast<void*>(&US1)) != 0) {
        std::cerr << "Thread creation failed. Exiting..." << std::endl;
        return -1;
    }

    if (pthread_create(&thread2, nullptr, ThreadFunction, reinterpret_cast<void*>(&US2)) != 0) {
        std::cerr << "Thread creation failed. Exiting..." << std::endl;
        return -1;
    }

    if (pthread_create(&thread3, nullptr, ThreadFunction, reinterpret_cast<void*>(&US3)) != 0) {
        std::cerr << "Thread creation failed. Exiting..." << std::endl;
        return -1;
    }

    if (pthread_create(&bluetoothThread, nullptr, BluetoothControlThread, reinterpret_cast<void*>(&serial_port)) != 0) {
        std::cerr << "Thread creation failed. Exiting..." << std::endl;
        return -1;
    }

    while (running) {
        // Do nothing
    }

    pthread_join(thread1, nullptr);
    pthread_join(thread2, nullptr);
    pthread_join(thread3, nullptr);
    pthread_join(bluetoothThread, nullptr);

    gpioSetMode(IN1, PI_OUTPUT);
    gpioSetMode(IN2, PI_OUTPUT);
    gpioSetMode(IN3, PI_OUTPUT);
    gpioSetMode(IN4, PI_OUTPUT);
    gpioSetMode(ENABLE, PI_OUTPUT);
    gpioSetMode(ENB, PI_OUTPUT);

    gpioSetPWMfrequency(ENABLE, 50);
    gpioSetPWMfrequency(ENB, 50);

    serClose(serial_port);
    gpioTerminate();

    return 0;
}
