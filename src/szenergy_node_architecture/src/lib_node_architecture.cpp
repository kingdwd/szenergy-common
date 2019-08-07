
#include <szenergy_node_architecture/node_statemachine.h>

/// Node state machine

bool NodeStateMachine::transitRunning()
{
    if (state==NODE_STATE::START)
    {
        state = NODE_STATE::RUNNING;
        return true;
    }
    return false;
}

bool NodeStateMachine::transitError()
{
    if (state!=NODE_STATE::START)
    {
        state = NODE_STATE::ERROR;
        return true;
    }
    return false;
}

bool NodeStateMachine::transitShutdown()
{
    if (state==NODE_STATE::RUNNING || state==NODE_STATE::ERROR)
    {
        state = NODE_STATE::SHUTDOWN;
        return true;
    }
    return false;
}

bool NodeStateMachine::transitContinueFromError()
{
    if (state==NODE_STATE::ERROR)
    {
        state = NODE_STATE::RUNNING;
        return true;
    }
    return false;
}

bool NodeStateMachine::transitReset()
{
    if (state==NODE_STATE::ERROR)
    {
        state = NODE_STATE::START;
        return true;
    }
    return false;
}

/// Port state machine

bool PortStateMachine::transitRunning()
{
    if (state==PORT_STATE::WAITING && parent->isRunning())
    {
        state = PORT_STATE::RUNNING;
        return true;
    }
    return false;
}

bool PortStateMachine::transitInitialize()
{
    if (state==PORT_STATE::INIT && parent->isRunning())
    {
        state = PORT_STATE::WAITING;
        return true;
    }
    return false;
}

bool PortStateMachine::transitError()
{
    if (state==PORT_STATE::RUNNING && parent->isRunning())
    {
        state = PORT_STATE::ERROR;
        return true;
    }
    return false;
}

bool PortStateMachine::transitContinueFromError()
{
    if (state==PORT_STATE::ERROR && parent->isRunning())
    {
        state = PORT_STATE::RUNNING;
        return true;
    }
    return false;
}

bool PortStateMachine::transitReset()
{
    if (state==PORT_STATE::ERROR && parent->isRunning())
    {
        state = PORT_STATE::INIT;
        return true;
    }
    return false;
}