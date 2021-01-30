/*
 * Example to demonstrate thread definition, semaphores, and thread sleep.
 */
#include <FreeRTOS_TEENSY4.h>
#include <imu.h>

// The LED is attached to pin 13 on the Teensy 4.0
const uint8_t LED_PIN = 13;

// Declare a semaphore handle.
SemaphoreHandle_t sem;
//------------------------------------------------------------------------------
/*
 * Thread 1, turn the LED off when signalled by thread 2.
 */
// Declare the thread function for thread 1.

static void printGyroState(ImuFiltered *gyroState, float dt)
{
    Serial.print("PITCH: \t");
    Serial.print(gyroState->pitch_IMU);
    Serial.print(" ");
    Serial.print("ROLL: \t");
    Serial.print(gyroState->roll_IMU);
    Serial.print(" ");
    Serial.print("YAW: \t");
    Serial.print(gyroState->yaw_IMU);
    Serial.println();
}

static void IMUWorker(void *arg)
{
    Serial.println("Pre cal");
    float SEC_DIVIS = 1000000.0;
    float dt = 0;
    float prevTime = micros();
    float currTime = micros();
    ImuFiltered imuData = {1., 1., 0};
    dt = (currTime - prevTime) / SEC_DIVIS;
    printGyroState(&imuData, dt);

    while (1) {
        Serial.println("Warmed up");
        currTime = micros();
        dt = (currTime - prevTime) / SEC_DIVIS;
        prevTime = currTime;
        getIMUdata(&imuData, dt);
        printGyroState(&imuData, dt);
        
        // vTaskDelay((200L * configTICK_RATE_HZ) / 1000L);

        // Wait for signal from thread 2.
        // xSemaphoreTake(sem, portMAX_DELAY);
    }
}

void setup()
{
    portBASE_TYPE imuWorkerInit;

    Serial.begin(9600);
    delay(4000);

    // Warm up IMU:
    IMUinit();
    calibrateAttitude();
    calculate_IMU_error();

    // initialize semaphore
    // sem = xSemaphoreCreateCounting(1, 0);

    // create task at priority two
    // imuWorkerInit = xTaskCreate(IMUWorker, NULL, configMINIMAL_STACK_SIZE, NULL, 2, NULL);

    // check for creation errors
    // if (imuWorkerInit != pdPASS)
    // {
    //     Serial.println("IMU Worker failed to initialize");
    //     while (1)
    //         ;
    // }

    Serial.println("Starting the scheduler !");

    // start scheduler
    // vTaskStartScheduler();
    // Serial.println("Insufficient RAM");
    // while (1)
    //     ;
}
//------------------------------------------------------------------------------
// WARNING idle loop has a very small stack (configMINIMAL_STACK_SIZE)
// loop must never block
void loop() {
    Serial.println("Pre cal");
    float SEC_DIVIS = 1000000.0;
    float dt = 0;
    float prevTime = micros();
    float currTime = micros();
    ImuFiltered imuData = {1., 1., 0};
    dt = (currTime - prevTime) / SEC_DIVIS;
    printGyroState(&imuData, dt);

    Serial.println("Warmed up");
    int c = 0;
    while (1) {
        currTime = micros();
        dt = (currTime - prevTime) / SEC_DIVIS;
        prevTime = currTime;
        getIMUdata(&imuData, dt);
        // printGyroState(&imuData, dt);
        delayMicroseconds(500);
        if (c % 500 == 0) printGyroState(&imuData, dt);
        c++;
    }
    // Not used.
}