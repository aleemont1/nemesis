#include "RocketFSM.hpp"
#include <config.h>

RocketFSM::RocketFSM() : currentState(RocketState::INACTIVE), previousState(RocketState::INACTIVE),
                         stateStartTime(0), lastUpdateTime(0)
{
}

void RocketFSM::init()
{
    setupStateActions();
    setupTransitions();
    currentState = RocketState::INACTIVE;
    stateStartTime = millis();
    lastUpdateTime = millis();

    if (stateActions[currentState].onEntry)
    {
        stateActions[currentState].onEntry();
    }
}

void RocketFSM::update()
{
    unsigned long currentTime = millis();
    lastUpdateTime = currentTime;

    if (stateActions[currentState].onUpdate)
    {
        stateActions[currentState].onUpdate();
    }

    checkTransitions();
}

FlightPhase RocketFSM::getCurrentPhase() const
{
    if (currentState == RocketState::CALIBRATING ||
        currentState == RocketState::READY_FOR_LAUNCH)
    {
        return FlightPhase::PRE_FLIGHT;
    }

    if (currentState >= RocketState::LAUNCH &&
        currentState <= RocketState::APOGEE)
    {
        return FlightPhase::FLIGHT;
    }

    if (currentState >= RocketState::STABILIZATION &&
        currentState <= RocketState::RECOVERED)
    {
        return FlightPhase::RECOVERY;
    }

    return FlightPhase::PRE_FLIGHT; // Default case
}

String RocketFSM::getStateString(RocketState state) const
{
    switch (state)
    {
    case RocketState::INACTIVE:
        return "INACTIVE";
    case RocketState::CALIBRATING:
        return "CALIBRATING";
    case RocketState::READY_FOR_LAUNCH:
        return "READY_FOR_LAUNCH";
    case RocketState::LAUNCH:
        return "LAUNCH";
    case RocketState::ACCELERATED_FLIGHT:
        return "ACCELERATED_FLIGHT";
    case RocketState::BALLISTIC_FLIGHT:
        return "BALLISTIC_FLIGHT";
    case RocketState::APOGEE:
        return "APOGEE";
    case RocketState::STABILIZATION:
        return "STABILIZATION";
    case RocketState::DECELERATION:
        return "DECELERATION";
    case RocketState::LANDING:
        return "LANDING";
    case RocketState::RECOVERED:
        return "RECOVERED";
    default:
        return "NULL";
    }
}

void RocketFSM::forceTransition(RocketState newState)
{
    this->transitionTo(newState);
}

void RocketFSM::setupStateActions()
{
    stateActions[RocketState::INACTIVE] = StateActions(
        [this]()
        { onInactiveEntry(); });

    stateActions[RocketState::CALIBRATING] = StateActions(
        [this]()
        { onCalibratingEntry(); },
        nullptr,
        [this]()
        { onCalibratingExit(); });

    stateActions[RocketState::READY_FOR_LAUNCH] = StateActions(
        [this]()
        { onReadyForLaunchEntry(); });

    stateActions[RocketState::LAUNCH] = StateActions(
        [this]()
        { onLaunchEntry(); });

    stateActions[RocketState::ACCELERATED_FLIGHT] = StateActions(
        [this]()
        { onAcceleratedFlightEntry(); },
        [this]()
        { onFlightUpdate(); });

    stateActions[RocketState::BALLISTIC_FLIGHT] = StateActions(
        [this]()
        { onBallisticFlightEntry(); },
        [this]()
        { onFlightUpdate(); });

    stateActions[RocketState::APOGEE] = StateActions(
        [this]()
        { onApogeeEntry(); });

    stateActions[RocketState::STABILIZATION] = StateActions(
        [this]()
        { onStabilizationEntry(); },
        [this]()
        { onRecoveryUpdate(); });

    stateActions[RocketState::DECELERATION] = StateActions(
        [this]()
        { onDecelerationEntry(); },
        [this]()
        { onRecoveryUpdate(); });

    stateActions[RocketState::LANDING] = StateActions(
        [this]()
        { onLandingEntry(); },
        nullptr,
        [this]()
        { onLandingExit(); });

    stateActions[RocketState::RECOVERED] = StateActions(
        [this]()
        { onRecoveredEntry(); },
        nullptr,
        [this]()
        { onRecoveredExit(); });
}

void RocketFSM::setupTransitions()
{
    transitions.emplace_back(RocketState::INACTIVE, RocketState::CALIBRATING, [this]()
                             { return true; }); // For now, automatically transition to calibrating mode. In future: wait for user input before starting calibration.
    transitions.emplace_back(RocketState::CALIBRATING, RocketState::READY_FOR_LAUNCH, [this]()
                             { return isCalibrationComplete(); });
    transitions.emplace_back(RocketState::READY_FOR_LAUNCH, RocketState::LAUNCH, [this]()
                             { return isLaunchDetected(); });
    transitions.emplace_back(RocketState::LAUNCH, RocketState::ACCELERATED_FLIGHT, [this]()
                             { return isLiftoffStarted(); });
    transitions.emplace_back(RocketState::ACCELERATED_FLIGHT, RocketState::BALLISTIC_FLIGHT, [this]()
                             { return isAccelerationPhaseComplete(); });
    transitions.emplace_back(RocketState::BALLISTIC_FLIGHT, RocketState::APOGEE, [this]()
                             { return isApogeeReached(); });
    transitions.emplace_back(RocketState::APOGEE, RocketState::STABILIZATION, [this]()
                             { return isDrogueReady(); });
    transitions.emplace_back(RocketState::STABILIZATION, RocketState::DECELERATION, [this]()
                             { return isStabilizationComplete(); });
    transitions.emplace_back(RocketState::DECELERATION, RocketState::LANDING, [this]()
                             { return isDecelerationComplete(); });
    transitions.emplace_back(RocketState::LANDING, RocketState::RECOVERED, [this]()
                             { return isLandingComplete(); });
}

void RocketFSM::transitionTo(RocketState newState)
{
    if (newState == currentState)
    {
        return;
    }

    if (stateActions[currentState].onExit)
    {
        stateActions[currentState].onExit();
    }

    previousState = currentState;
    currentState = newState;
    stateStartTime = millis();

    if (stateActions[currentState].onEntry)
    {
        stateActions[currentState].onEntry();
    }
}

void RocketFSM::checkTransitions()
{
    for (const auto &transition : transitions)
    {
        if (transition.fromState == currentState && transition.condition())
        {
            transitionTo(transition.toState);
            break; // One transition per update cycle
        }
    }
}

void RocketFSM::onInactiveEntry()
{
    // Start initialization task
}

void RocketFSM::onCalibratingEntry()
{
    // Start calibrating task
    // Start system health check task
}

void RocketFSM::onReadyForLaunchEntry()
{
    // Start logger task
    // Start telemetry task
}

void RocketFSM::onLaunchEntry()
{
    // Start flight timer
}

void RocketFSM::onAcceleratedFlightEntry()
{
    // Start EKF elaboration
}

void RocketFSM::onBallisticFlightEntry()
{
    // Start apogee detection task
}

void RocketFSM::onApogeeEntry()
{
    // Start drogue 'chute opening task
}

void RocketFSM::onStabilizationEntry()
{
    // Start main 'chute opening task
}

void RocketFSM::onDecelerationEntry()
{

}

void RocketFSM::onLandingEntry()
{
    // Stop EKF elaboration
    // Stop data collection task
}

void RocketFSM::onRecoveredEntry()
{
    // Stop telemetry task
}

void RocketFSM::onCalibratingExit()
{
    // Health checks are ok, calibration is ok
    // Start reading data task
}

void RocketFSM::onLandingExit()
{
    // Save logs on SD card
}

void RocketFSM::onRecoveredExit()
{
    // Stop all tasks and make some happy noises
}

void RocketFSM::onFlightUpdate()
{
}

void RocketFSM::onRecoveryUpdate()
{
    
}

bool RocketFSM::isCalibrationComplete()
{
    return false;
}

bool RocketFSM::isSystemReady()
{
    return false;
}

bool RocketFSM::isLaunchDetected()
{
    // Acceleration > Acceleration_threshold for at least liftoff_timeout_threshold ms
    return false;
}

bool RocketFSM::isLiftoffStarted()
{
    return millis() - stateStartTime > LIFTOFF_TIMEOUT_MS; // Detect liftoff after 100ms from launch (previous condition satisfied)
}

bool RocketFSM::isAccelerationPhaseComplete()
{
    // Acceleration = 0 for at least ballistic_timeout_threshold ms
    return false;
}

bool RocketFSM::isBallisticPhaseComplete()
{
    // Vertical velocity is ~0 for at least apogee_timeout_threshold ms
    return false;
}

bool RocketFSM::isApogeeReached()
{
    // Vertical velocity is < 0 (falling)
    return false;
}

bool RocketFSM::isDrogueReady()
{
    return millis() - stateStartTime > DROGUE_APOGEE_TIMEOUT; // Open drogue after 300ms
}

bool RocketFSM::isStabilizationComplete()
{
    // Altitude is < main_parachute_altitude_threshold meters
    return false;
}

bool RocketFSM::isDecelerationComplete()
{
    // Reached minimum vertical velocity before landing 
    return false;
}

bool RocketFSM::isLandingComplete()
{
    // Vertical velocity is 0 for at least landing_timout_threshold ms
    return false;
}
