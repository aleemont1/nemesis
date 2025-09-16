#pragma once

#include "FlightState.hpp"
#include <ILogger.hpp>
#include <ISensor.hpp>
#include <ITransmitter.hpp>
#include <Arduino.h>
#include <map>
#include <vector>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <config.h>
#include <KalmanFilter1D.hpp>

// Shared sensor data structure
struct SharedSensorData
{
    class SensorData imuData;
    class SensorData baroData1;
    class SensorData baroData2;
    class SensorData gpsData;
    uint32_t timestamp;
    bool dataValid;
};

struct SharedFilteredData
{
    float altitude;
    float verticalVelocity;
    float orientation[4]; // Quaternion [w, x, y, z]
    uint32_t timestamp;
    bool dataValid;
};

/**
 * @brief Rocket Finite State Machine class with FreeRTOS integration
 */
class RocketFSM
{
public:
    RocketFSM();
    ~RocketFSM();

    /**
     * @brief Initialize the FSM and set up states and transitions
     */
    void init();

    /**
     * @brief Start the FSM task
     */
    void start();

    /**
     * @brief Stop the FSM task
     */
    void stop();

    /**
     * @brief Send an event to the FSM
     */
    bool sendEvent(FSMEvent event, RocketState targetState = RocketState::INACTIVE, void *eventData = nullptr);

    /**
     * @brief Get current state (thread-safe)
     */
    RocketState getCurrentState();

    /**
     * @brief Get current flight phase
     */
    FlightPhase getCurrentPhase();

    /**
     * @brief Get state name as string for debugging
     */
    String getStateString(RocketState state) const;

    /**
     * @brief Force a state transition (for testing/emergency)
     */
    void forceTransition(RocketState newState);

    /**
     * @brief Check if FSM is in final state
     */
    bool isFinished();

private:
    // FSM Configuration constants
    static const int EVENT_QUEUE_SIZE = 10;
    static const int FSM_TASK_STACK_SIZE = 4096;
    static const UBaseType_t FSM_TASK_PRIORITY = 2; // Highest priority
    // FreeRTOS components
    TaskHandle_t fsmTaskHandle;
    QueueHandle_t eventQueue;
    SemaphoreHandle_t stateMutex;

    // Task handles for different subsystems
    TaskHandle_t sensorTaskHandle;          // Core 0 - Critical
    TaskHandle_t ekfTaskHandle;             // Core 0 - Critical
    TaskHandle_t apogeeDetectionTaskHandle; // Core 0 - Critical
    TaskHandle_t recoveryTaskHandle;        // Core 0 - Critical

    TaskHandle_t dataCollectionTaskHandle; // Core 1 - Non-critical
    TaskHandle_t telemetryTaskHandle;      // Core 1 - Non-critical
    TaskHandle_t gpsTaskHandle;            // Core 1 - Non-critical
    TaskHandle_t loggingTaskHandle;        // Core 1 - Non-critical

    // State management
    RocketState currentState;
    RocketState previousState;
    unsigned long stateStartTime;
    unsigned long lastUpdateTime;
    bool isRunning;

    std::map<RocketState, StateActions> stateActions;
    std::vector<StateTransition> transitions;

private:
    SharedSensorData sharedData;
    /**
     * @brief Static wrapper for FreeRTOS FSM task
     */
    static void fsmTaskWrapper(void *parameter);

    /**
     * @brief Main FSM task function
     */
    void fsmTask();

    /**
     * @brief Setup all state actions and task configurations
     */
    void setupStateActions();

    /**
     * @brief Setup all state transitions
     */
    void setupTransitions();

    /**
     * @brief Execute state transition
     */
    void transitionTo(RocketState newState);

    /**
     * @brief Process incoming events
     */
    void processEvent(const FSMEventData &eventData);

    /**
     * @brief Start tasks for current state
     */
    void startStateTasks();

    /**
     * @brief Stop all running tasks
     */
    void stopAllTasks();

    /**
     * @brief Check transition conditions (called by sensor task)
     */
    void checkTransitions();

    /* These static wrappers are needed because FreeRTOS tasks require a C-like signature
        for Task creation. These are wrappers of the C++ class methods below. */
    // Static task wrappers for Core 0 (Critical)
    static void sensorTaskWrapper(void *parameter);
    static void ekfTaskWrapper(void *parameter);
    static void apogeeDetectionTaskWrapper(void *parameter);
    static void recoveryTaskWrapper(void *parameter);

    // Static task wrappers for Core 1 (Non-critical)
    static void dataCollectionTaskWrapper(void *parameter);
    static void telemetryTaskWrapper(void *parameter);
    static void gpsTaskWrapper(void *parameter);
    static void loggingTaskWrapper(void *parameter);

    // Task functions for Core 0 (Critical)
    /* Collect IMU and Barometers data */
    void sensorTask();
    /* Run the EKF to fuse sensor data */
    void ekfTask();
    /* Detect apogee based on EKF data */
    void apogeeDetectionTask();
    /* Handle recovery system deployment */
    void recoveryTask();

    // Task functions for Core 1 (Non-critical)
    /* Aggregate sensor data and GPS data in the shared data structure (might be removed)*/
    void dataCollectionTask();
    /* Send telemetry data via LoRa */
    void telemetryTask();
    /* Read GPS data and update shared structure */
    void gpsTask();
    /* Log data to SD card */
    void loggingTask();

    // State entry/exit actions
    void onInactiveEntry();
    void onCalibratingEntry();
    void onReadyForLaunchEntry();
    void onLaunchEntry();
    void onAcceleratedFlightEntry();
    void onBallisticFlightEntry();
    void onApogeeEntry();
    void onStabilizationEntry();
    void onDecelerationEntry();
    void onLandingEntry();
    void onRecoveredEntry();

    void onCalibratingExit();
    void onLandingExit();
    void onRecoveredExit();

    // Condition checking functions
    bool isCalibrationComplete();
    bool isSystemReady();
    bool isLaunchDetected();
    bool isLiftoffStarted();
    bool isAccelerationPhaseComplete();
    bool isBallisticPhaseComplete();
    bool isApogeeReached();
    bool isDrogueReady();
    bool isStabilizationComplete();
    bool isDecelerationComplete();
    bool isLandingComplete();
};