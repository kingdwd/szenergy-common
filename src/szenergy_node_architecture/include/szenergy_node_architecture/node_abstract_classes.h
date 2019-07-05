#ifndef NODE_ABSTRACT_CLASSES_H
#define NODE_ABSTRACT_CLASSES_H

#include <vector>

#include <ros/ros.h>

#include "node_statemachine.h"


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