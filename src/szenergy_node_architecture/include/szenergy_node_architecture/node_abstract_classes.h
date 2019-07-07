#ifndef NODE_ABSTRACT_CLASSES_H
#define NODE_ABSTRACT_CLASSES_H

#include <vector>

#include <ros/ros.h>

#include "node_statemachine.h"

enum class CONTROLSTATE_STATES {START, SYNCHRONIZED, DISABLED };

class AbstractNodeState
{
protected:
	CONTROLSTATE_STATES state;
	std::vector<std::shared_ptr<PortStateMachine> > synchronizing_sm;
public:
	AbstractNodeState(): state(CONTROLSTATE_STATES::START)
	{

	}

	void addSyncPortStateMachine(std::shared_ptr<PortStateMachine> sm)
	{
		synchronizing_sm.push_back(sm);
	}

	bool synchronized()
	{
		bool is_synchronized = true;
		for (const auto& sm: synchronizing_sm)
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
		if (state == CONTROLSTATE_STATES::START)
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
		}
		return false;
	}

	void transform()
	{
		switch(state)
		{
			case CONTROLSTATE_STATES::START:
			{
				transitSynchronizing();
				break;
			}
			case CONTROLSTATE_STATES::SYNCHRONIZED:
			{
				transformFunc();
				break;
			}
			case CONTROLSTATE_STATES::DISABLED:
			{
				break;
			}
		}
	}

	virtual void transformFunc() = 0;
};

class AbstractStateNode
{
private:
protected:
    std::shared_ptr<ros::NodeHandle> nh;
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
    AbstractStateNode(std::shared_ptr<ros::NodeHandle> nh): 
        nh(nh),
        node_state_machine(new NodeStateMachine())
    {}
};

#endif
