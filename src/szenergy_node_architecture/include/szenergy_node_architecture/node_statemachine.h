#ifndef NODE_STATEMACHINE_H
#define NODE_STATEMACHINE_H

#include <memory>

enum class NODE_STATE {START, RUNNING, ERROR, SHUTDOWN};

enum class PORT_STATE {INIT, RUNNING, ERROR};

class NodeStateMachine 
{
private:
    NODE_STATE state;
protected:
public:
    NodeStateMachine(): state(NODE_STATE::START) {}
    inline bool isRunning()
    {
        return state==NODE_STATE::RUNNING;
    }
    inline bool isError()
    {
        return state==NODE_STATE::ERROR;
    }
    inline bool isShuttingDown()
    {
        return state==NODE_STATE::SHUTDOWN;
    }
    inline bool isStart()
    {
        return state==NODE_STATE::START;
    }
    bool transitRunning();              /// Setup complete, node is running
    bool transitError();                /// Error caught
    bool transitShutdown();             /// Shut down node
    /// Error handling
    bool transitReset();                /// Re-initialize
    bool transitContinueFromError();    /// Continue from error state
};

class PortStateMachine
{
private:
    PORT_STATE state;                           /// State
    std::shared_ptr<NodeStateMachine> parent;   /// Parent state machine
protected:
    
public:
    PortStateMachine(std::shared_ptr<NodeStateMachine> parent): 
        state(PORT_STATE::INIT), parent(parent) {}

    /// State related functions
    inline bool isRunning()
    {
        return state==PORT_STATE::RUNNING && parent->isRunning();
    }
    inline bool isError()
    {
        return state==PORT_STATE::ERROR || parent->isError();
    }
    inline bool isStart()
    {
        return state==PORT_STATE::INIT && parent->isRunning();
    }
    bool transitRunning();              /// Setup of subscriber is complete
    bool transitError();                /// Error caught
    /// Error handling
    bool transitReset();                /// Re-initialize
    bool transitContinueFromError();    /// Continue from error state
};

#endif