
#include <szenergy_node_architecture/node_statemachine.h>
#include <szenergy_node_architecture/node_abstract_classes.h>

/// Node state machine

namespace szenergy
{

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

bool AbstractStateNode::setAllSubPortStateMachineRunning()
{
	for (const auto &sm: sub_port_state_machines)
	{
		if (!sm->transitInitialize())
		{
			return false;
		}
	}
	return true;
}

bool AbstractNodeState::transitSynchronizing()
{
	switch(state)
	{
		case CONTROLSTATE_STATES::WAITING:
		{
			if (synchronized())
			{
				state = CONTROLSTATE_STATES::SYNCHRONIZED;
				return true;
			}
			else
			{
				return false;
			}
			break;
		}
		default:
		{
			return false;
		}
	}
	return false;
}

bool AbstractNodeState::transitDanger()
{
	switch(state)
	{
		case CONTROLSTATE_STATES::SYNCHRONIZED:
		{
			state = CONTROLSTATE_STATES::DANGER;
			return true;
			break;
		}
		default:
		{
			return false;
			break;
		}
	}
}

void AbstractNodeState::transform()
{
	switch(state)
	{
		case CONTROLSTATE_STATES::WAITING:
		{
			transitSynchronizing();
			break;
		}
		case CONTROLSTATE_STATES::SYNCHRONIZED:
		{
			transformFunc();
			break;
		}
		case CONTROLSTATE_STATES::DANGER:
		{
			handlingDanger();
			break;
		}
		case CONTROLSTATE_STATES::DISABLED:
		{
			break;
		}
	}
}

bool AbstractNodeState::transitDangerUnhandled()
{
	switch(state)
	{
		case CONTROLSTATE_STATES::DANGER:
		{
			transitDisabled();
			synchronizing_hierarch_sm->transitError();
			break;
		}
	}
}

bool AbstractNodeState::synchronized()
{
	bool is_synchronized = true;
	for (const auto& sm: synchronizing_port_sm)
	{
		if (!sm->isRunning())
		{
			is_synchronized &= 0;
			return is_synchronized;
		}
	}

	return is_synchronized;
}

bool AbstractNodeState::transitWaiting()
{
	switch(state)
	{
		case CONTROLSTATE_STATES::DANGER:
		{
			state = CONTROLSTATE_STATES::WAITING;
			return true;
			break;
		}
		default:
		{
			return false;
			break;
		}
	}
}

bool AbstractNodeState::transitDangerHandled()
{
	switch(state)
	{
		case CONTROLSTATE_STATES::DANGER:
		{
			transitWaiting();
			return true;
			break;
		}
		default:
		{
			return false;
		}
	}
}

bool AbstractNodeState::transitDisabled()
{
	switch(state)
	{
		case CONTROLSTATE_STATES::SYNCHRONIZED:
		case CONTROLSTATE_STATES::DANGER:
		{
			state = CONTROLSTATE_STATES::DISABLED;
			break;
		}
	}
}

}
