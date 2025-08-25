#pragma once
#include <functional>
/**
 * @brief Enum representing all possible rocket states
 */
enum class RocketState
{
    // Initial state
    INACTIVE,

    // Pre-flight phase
    CALIBRATING,
    READY_FOR_LAUNCH,

    // Flight phase
    LAUNCH,
    ACCELERATED_FLIGHT,
    BALLISTIC_FLIGHT,
    APOGEE,

    // Recovery phase
    STABILIZATION,
    DECELERATION,
    LANDING,
    RECOVERED
};

/**
 * @brief Enum representing flight phases for better organization
 */
enum class FlightPhase
{
    PRE_FLIGHT,
    FLIGHT,
    RECOVERY
};


/**
 * @brief Structure to hold state transition information
 * 
 */
struct StateTransition
{
    RocketState fromState;
    RocketState toState;
    std::function<bool()> condition;    // Lambda function to check transition conditions

    StateTransition(RocketState from, RocketState to, const std::function<bool()>& cond)
        : fromState(from), toState(to), condition(cond) {}
};

/**
 * @brief Structure to hold state actions
 * This structure allows defining actions to be executed on state entry, update, and exit.
 * It can be used to encapsulate the behavior associated with each state.
 * The actions are lambdas that can be defined when creating the state machine.
 */
struct StateActions
{
    std::function<void()> onEntry;
    std::function<void()> onUpdate;
    std::function<void()> onExit;

    StateActions() : onEntry(nullptr), onUpdate(nullptr), onExit(nullptr) {}
    StateActions(const std::function<void()>& entry, const std::function<void()>& update = nullptr, const std::function<void()>& exit = nullptr)
        : onEntry(entry), onUpdate(update), onExit(exit) {}
};