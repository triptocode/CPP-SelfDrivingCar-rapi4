#include <pigpio.h>
#include <iostream>
#include <csignal>
#include <pthread.h>
#include <thread>
#include <chrono>
#include "motor_control.h"
#include <unistd.h>


bool on_flag = false;
using namespace std;

bool running = true;

const char* device = "/dev/ttyS0";
const int baudrate = 9600;

char* mutableDevice = const_cast<char*>(device);
int serial_port = serOpen(mutableDevice, baudrate, 0);


class UltraSound
{
public:
    void Init(unsigned int trigger, unsigned int echo, int sensorNumber);
    double GetDistance(unsigned int timeout);
    int GetSensorNumber() const;
    double GetLatestDistance() const;
    void SetLatestDistance(double distance);

private:
    void RecordPulseLength();
    int trigger;
    int echo;
    int sensorNumber;

    volatile long startTimeUsec;
    volatile long endTimeUsec;

    double distanceCm;
    double previousDistanceCm; 
    long DifferenceTimeUsec;
    long now;

    pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER; // Initialize mutex

    static const double DISTANCE_CONVERSION_FACTOR;
};

void *ThreadFunction(void *arg);

const double UltraSound::DISTANCE_CONVERSION_FACTOR = 58.7734;

void UltraSound::Init(unsigned int trigger, unsigned int echo, int sensorNumber)
{
    this->trigger = trigger;
    this->echo = echo;
    this->sensorNumber = sensorNumber;

    gpioSetMode(trigger, PI_OUTPUT);
    gpioSetMode(echo, PI_INPUT);
    gpioWrite(trigger, 0);
    gpioSleep(PI_TIME_RELATIVE, 0.5, 0);

    // Initialize mutex
    if (pthread_mutex_init(&mutex, nullptr) != 0)
    {
        cerr << "Mutex initialization failed. Exiting..." << endl;
        exit(-1);
    }
}

double UltraSound::GetDistance(unsigned int timeout)
{
    pthread_mutex_lock(const_cast<pthread_mutex_t*>(&mutex)); // Lock mutex before accessing shared resource

    endTimeUsec = 0;
    gpioWrite(trigger, 1);
    gpioDelay(10);
    gpioWrite(trigger, 0);

    now = gpioTick();

    while (endTimeUsec == 0 && gpioTick() - now < timeout)
    {
        RecordPulseLength();
    }

    DifferenceTimeUsec = endTimeUsec - startTimeUsec;
    distanceCm = (double)DifferenceTimeUsec / DISTANCE_CONVERSION_FACTOR;

    // 
    if (distanceCm == previousDistanceCm) {
        pthread_mutex_unlock(const_cast<pthread_mutex_t*>(&mutex)); // Unlock mutex after accessing shared resource
        return distanceCm;
    }

    previousDistanceCm = distanceCm; // 

    pthread_mutex_unlock(const_cast<pthread_mutex_t*>(&mutex)); // Unlock mutex after accessing shared resource

    if (endTimeUsec != 0)
        return distanceCm;
    else
        return 0;
}
double UltraSound::GetLatestDistance() const
{
    // Lock mutex before accessing shared resource
    pthread_mutex_lock(const_cast<pthread_mutex_t*>(&mutex));

    double latestDistance = distanceCm;

    // Unlock mutex after accessing shared resource
    pthread_mutex_unlock(const_cast<pthread_mutex_t*>(&mutex));

    return latestDistance;
}
void UltraSound::SetLatestDistance(double distance)
{
    // Lock mutex before updating shared resource
    pthread_mutex_lock(const_cast<pthread_mutex_t*>(&mutex));

    distanceCm = distance;

    // Unlock mutex after updating shared resource
    pthread_mutex_unlock(const_cast<pthread_mutex_t*>(&mutex));
}

void UltraSound::RecordPulseLength()
{
    startTimeUsec = gpioTick();
    while (gpioRead(echo) == 1)
    {
        endTimeUsec = gpioTick();
    }
}

int UltraSound::GetSensorNumber() const
{
    return sensorNumber;
}

void *ThreadFunction(void *arg) {
    UltraSound *us1 = new UltraSound();
    us1->Init(0, 1, 1);

    UltraSound *us2 = new UltraSound();
    us2->Init(5, 6, 2);

    UltraSound *us3 = new UltraSound();
    us3->Init(26, 16, 3);

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

        if (on_flag) {
            // Check distance and perform left or right turn logic
            if (distance2 >= 50.0) {
                std::cout << "Performing left turn" << std::endl;
                Driving_left(IN1, IN2, IN3, IN4, 80);
                gpioPWM(ENABLE, 128); // Enable PWM
                gpioPWM(ENB, 128);
            }
            if (distance2 >= 20.0 && distance2 <50) {
                std::cout << "Continuing forward" << std::endl;
                Driving_forward(IN1, IN2, IN3, IN4, 80);
                gpioPWM(ENABLE, 128); // Enable PWM
                gpioPWM(ENB, 128);

            }
            if (distance2 < 20)
            {
                if(distance1 > distance3)
                {
                        std::cout << "turn left" << std::endl;
                        Driving_left(IN1, IN2, IN3, IN4, 100);
                        gpioPWM(ENABLE, 128); // Enable PWM
                        gpioPWM(ENB, 128);

                }
                else if (distance1 < distance3)
		{
		 	std::cout << "Continuing forward" << std::endl;
                        Driving_forward(IN1, IN2, IN3, IN4, 100);
                        gpioPWM(ENABLE, 128); // Enable PWM
                        gpioPWM(ENB, 128);
                } 
                else {
                        std::cout << "Continuing backward" << std::endl;
                        Driving_backward(IN1, IN2, IN3, IN4, 80);
                        gpioPWM(ENABLE, 128); // Enable PWM
                        gpioPWM(ENB, 128);
                     }
            }

	    if (distance2 < 25.0 && distance1< 25.0 ) 
            {
                std::cout << "turn right" << std::endl;
                Driving_right(IN1, IN2, IN3, IN4, 100);
                gpioPWM(ENABLE, 128); // Enable PWM
                gpioPWM(ENB, 128);

            }
            if (distance2 < 25.0 && distance3 < 25.0)
            {
                std::cout << "turn left" << std::endl;
                Driving_left(IN1, IN2, IN3, IN4, 100);
                gpioPWM(ENABLE, 128); // Enable PWM
                gpioPWM(ENB, 128);
            }
            if (distance1 <25.0 && distance2< 25.0 && distance3 < 25.0)
            {
                std::cout << "go backward" << std::endl;
                Driving_backward(IN1, IN2, IN3, IN4, 100);
                gpioPWM(ENABLE, 128); // Enable PWM
                gpioPWM(ENB, 128);

            }
		
		if (distance1 < 25.0 )
            {
                std::cout << "turn right" << std::endl;
                Driving_right(IN1, IN2, IN3, IN4, 100);
                gpioPWM(ENABLE, 128); // Enable PWM
                gpioPWM(ENB, 128);

            }
            if (distance3 < 25 )
            {
                std::cout << "turn left" << std::endl;
                Driving_left(IN1, IN2, IN3, IN4, 100);
                gpioPWM(ENABLE, 128); // Enable PWM
                gpioPWM(ENB, 128);
            }
	}
      }


        // Add other conditions or actions based on the distance values if needed

    delete us1;
    delete us2;
    delete us3;

    return nullptr;
}
int calculateSpeed(double distance) {
    // Add your logic here to determine the speed based on the distance
    // You may want to experiment and adjust the parameters according to your needs
    if (distance < 10.0) {
        return 50;  // Adjust the speed for a close obstacle
    } else if (distance < 20.0) {
        return 100;  // Adjust the speed for a moderately close obstacle
    } else {
        return 128;  // Default speed
    }
}

// Function to control motors based on Bluetooth input
void controlMotors(char input, UltraSound& us1, UltraSound& us2, UltraSound& us3) {
    // Get the average distance from all three sensors
    double avgDistance = (us1.GetLatestDistance() + us2.GetLatestDistance() + us3.GetLatestDistance()) / 3.0;
    // Adjust the speed based on the distance
    int speed = calculateSpeed(avgDistance);

    // Add your motor control logic here
 
    // Stop motors if no valid input
    if (input == '2')
    {
    	std::cout << "Stopping motors" << std::endl;
    	Driving_stop(IN1, IN2, IN3, IN4);
    	gpioPWM(ENABLE, 0); // Disable PWM
    	gpioPWM(ENB, 0);
    	return;
    }
   
}

UltraSound US1, US2, US3;

void *BluetoothControlThread(void *arg) {
    int serial_port = *(int *)arg;  // Cast the argument back to int (assuming it's the serial port)

    try {
        while (running) {
            if (serDataAvailable(serial_port) > 0) {
                char input = serReadByte(serial_port);
                // Ensure the sensors are accessible globally or pass them as arguments
                if (input == '2' && on_flag) {
                    std::cout << "Stopping motors" << std::endl;
                    Driving_stop(IN1, IN2, IN3, IN4);
                    gpioPWM(ENABLE, 0); // Disable PWM
                    gpioPWM(ENB, 0);
                    on_flag = false;
                } else if (input == '1') {
                    on_flag = true;
                }
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Bluetooth communication error: " << e.what() << std::endl;
        // Handle the exception or exit gracefully
    }

    return nullptr;
}


// Handle Ctrl+C to gracefully exit
void signalHandler(int signum) {
    // Stop the program when Ctrl+C is pressed
    gpioPWM(ENABLE, 0); // Disable PWM
    gpioPWM(ENB, 0);
    gpioTerminate();
    exit(signum);
}
int main() {



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

    US1.Init(0, 1, 1);
    US2.Init(5, 6, 2);
    US3.Init(26, 16, 3);

    pthread_t thread1, thread2, thread3, bluetoothThread;

    if (pthread_create(&thread1, nullptr, ThreadFunction, &US1) != 0) {
        std::cerr << "Thread creation failed. Exiting..." << std::endl;
        return -1;
    }

    if (pthread_create(&thread2, nullptr, ThreadFunction, &US2) != 0) {
        std::cerr << "Thread creation failed. Exiting..." << std::endl;
        return -1;
    }

    if (pthread_create(&thread3, nullptr, ThreadFunction, &US3) != 0) {
        std::cerr << "Thread creation failed. Exiting..." << std::endl;
        return -1;
    }

    if (pthread_create(&bluetoothThread, nullptr, BluetoothControlThread, (void *)&serial_port) != 0) {
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

    // Set PWM frequency to 1000 Hz
    gpioSetPWMfrequency(ENABLE, 50);
    gpioSetPWMfrequency(ENB, 50);



    serClose(serial_port);
    gpioTerminate();

    return 0;
}
