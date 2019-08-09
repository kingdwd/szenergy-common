#ifndef NODE_ABSTRACT_CLASSES_H
#define NODE_ABSTRACT_CLASSES_H

#include <vector>

#include <ros/ros.h>

#include "node_statemachine.h"

enum class CONTROLSTATE_STATES {WAITING, SYNCHRONIZED, DANGER, DISABLED };

class AbstractNodeState
{
protected:
	CONTROLSTATE_STATES state;
	// Relation with state machines
	std::shared_ptr<NodeStateMachine> synchronizing_hierarch_sm;
	std::vector<std::shared_ptr<PortStateMachine> > synchronizing_port_sm;
public:
	AbstractNodeState(): state(CONTROLSTATE_STATES::WAITING)
	{
	}

	void setSyncParentStateMachine(std::shared_ptr<NodeStateMachine> sm)
	{
		synchronizing_hierarch_sm = sm;
	}

	void addSyncPortStateMachine(std::shared_ptr<PortStateMachine> sm)
	{
		synchronizing_port_sm.push_back(sm);
	}

	bool synchronized()
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

	bool transitSynchronizing()
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

	bool transitDanger()
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

	bool transitWaiting()
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

	bool transitDangerHandled()
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

	bool transitDisabled()
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

	bool transitDangerUnhandled()
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

	void transform()
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

	virtual void transformFunc() = 0;
	virtual void handlingDanger() = 0;
};

class AbstractStateNode
{
private:
protected:
    std::shared_ptr<ros::NodeHandle> nh;
    std::shared_ptr<ros::NodeHandle> private_nh;
    std::shared_ptr<NodeStateMachine> node_state_machine;
    std::vector<std::shared_ptr<PortStateMachine> > sub_port_state_machines; 
    std::vector<std::shared_ptr<PortStateMachine> > pub_port_state_machines; 

    bool setAllSubPortStateMachineRunning()
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
    bool processingAllowedOnPort(std::shared_ptr<PortStateMachine> p_sm)
    {
        return node_state_machine->isRunning() && (p_sm->isWaiting() || p_sm->isRunning());
    }
public:
    AbstractStateNode(std::shared_ptr<ros::NodeHandle> nh, std::shared_ptr<ros::NodeHandle> private_nh):
        nh(nh),
        node_state_machine(new NodeStateMachine())
    {}
};

#endif
