#include <gtest/gtest.h>


#include <szenergy_node_architecture/node_statemachine.h>

TEST(TestStateMachine, testNormalStateMachine)
{
    NodeStateMachine statemachine;
    statemachine.transitRunning();
    ASSERT_FALSE(statemachine.isError());
    ASSERT_TRUE(statemachine.isRunning());
    statemachine.transitShutdown();
    ASSERT_TRUE(statemachine.isShuttingDown());
    ASSERT_FALSE(statemachine.isRunning());
}

TEST(TestStateMachine, testErrorStateMachine)
{
    NodeStateMachine statemachine;
    statemachine.transitRunning();
    ASSERT_TRUE(statemachine.isRunning());
    statemachine.transitError();
    ASSERT_TRUE(statemachine.isError());
    ASSERT_FALSE(statemachine.isRunning());
    statemachine.transitShutdown();
    ASSERT_TRUE(statemachine.isShuttingDown());
    ASSERT_FALSE(statemachine.isError());
}

TEST(TestStateMachine, testStartNoImmediateErrorAndShutdown)
{
    NodeStateMachine statemachine;
    statemachine.transitError();
    ASSERT_FALSE(statemachine.isError());
    statemachine.transitShutdown();
    ASSERT_FALSE(statemachine.isShuttingDown());
}

TEST(TestStateMachine, testErrorHandling)
{
    NodeStateMachine statemachine;
    statemachine.transitRunning();
    statemachine.transitError();
    ASSERT_TRUE(statemachine.isError());
    statemachine.transitRunning();
    ASSERT_TRUE(statemachine.isError());
    ASSERT_FALSE(statemachine.isRunning());
}

TEST(TestStateMachine, testErrorHandlingContinue)
{
    NodeStateMachine statemachine;
    statemachine.transitRunning();
    statemachine.transitError();
    ASSERT_TRUE(statemachine.isError());
    statemachine.transitContinueFromError();
    ASSERT_FALSE(statemachine.isError());
    ASSERT_TRUE(statemachine.isRunning());
}

TEST(TestStateMachine, testErrorHandlingReset)
{
    NodeStateMachine statemachine;
    statemachine.transitRunning();
    statemachine.transitError();
    ASSERT_TRUE(statemachine.isError());
    statemachine.transitReset();
    ASSERT_FALSE(statemachine.isError());
    ASSERT_FALSE(statemachine.isRunning());
    ASSERT_TRUE(statemachine.isStart());
}

TEST(TestHierarchicalStateMachine, testBasicHierarchicalNotRunning)
{
    std::shared_ptr<NodeStateMachine> node_sm(new NodeStateMachine());
    PortStateMachine sm_0(node_sm);
    sm_0.transitRunning();
    ASSERT_FALSE(sm_0.isRunning());
}

TEST(TestHierarchicalStateMachine, testBasicHierarchicalRunning)
{
    std::shared_ptr<NodeStateMachine> node_sm(new NodeStateMachine());
    PortStateMachine sm_0(node_sm);
    node_sm->transitRunning();
    sm_0.transitInitialize();
    sm_0.transitRunning();
    ASSERT_TRUE(sm_0.isRunning());
}

TEST(TestHierarchicalStateMachine, testBasicHierarchicalError)
{
    std::shared_ptr<NodeStateMachine> node_sm(new NodeStateMachine());
    PortStateMachine sm_0(node_sm);
    node_sm->transitRunning();
    sm_0.transitInitialize();
    sm_0.transitRunning();    
    sm_0.transitError();
    ASSERT_FALSE(sm_0.isRunning());
    ASSERT_TRUE(sm_0.isError());
}

TEST(TestHierarchicalStateMachine, testBasicHierarchicalErrorHandling)
{
    std::shared_ptr<NodeStateMachine> node_sm(new NodeStateMachine());
    PortStateMachine sm_0(node_sm);
    node_sm->transitRunning();
    sm_0.transitInitialize();
    sm_0.transitRunning();
    sm_0.transitError();
    sm_0.transitRunning();
    ASSERT_FALSE(sm_0.isRunning());
    ASSERT_TRUE(sm_0.isError());
}

TEST(TestHierarchicalStateMachine, testBasicHierarchicalErrorHandlingReset)
{
    std::shared_ptr<NodeStateMachine> node_sm(new NodeStateMachine());
    PortStateMachine sm_0(node_sm);
    node_sm->transitRunning();
    sm_0.transitInitialize();
    sm_0.transitRunning();
    sm_0.transitError();
    sm_0.transitReset();
    ASSERT_FALSE(sm_0.isRunning());
    ASSERT_FALSE(sm_0.isError());
    ASSERT_TRUE(sm_0.isStart());
}

TEST(TestHierarchicalStateMachine, testBasicHierarchicalErrorHandlingContinue)
{
    std::shared_ptr<NodeStateMachine> node_sm(new NodeStateMachine());
    PortStateMachine sm_0(node_sm);
    node_sm->transitRunning();
    sm_0.transitInitialize();
    sm_0.transitRunning();
    sm_0.transitError();
    sm_0.transitContinueFromError();
    ASSERT_TRUE(sm_0.isRunning());
    ASSERT_FALSE(sm_0.isError());
}

TEST(TestHierarchicalStateMachine, testBasicHierarchicalParentError)
{
    std::shared_ptr<NodeStateMachine> node_sm(new NodeStateMachine());
    PortStateMachine sm_0(node_sm);
    node_sm->transitRunning();
    sm_0.transitInitialize();
    sm_0.transitRunning();
    node_sm->transitError();
    ASSERT_TRUE(sm_0.isError());
}

TEST(TestHierarchicalStateMachine, testBasicHierarchicalParentShutdown)
{
    std::shared_ptr<NodeStateMachine> node_sm(new NodeStateMachine());
    PortStateMachine sm_0(node_sm);
    node_sm->transitRunning();
    sm_0.transitInitialize();
    sm_0.transitRunning();
    node_sm->transitShutdown();
    ASSERT_FALSE(sm_0.isError());
    ASSERT_FALSE(sm_0.isRunning());
    ASSERT_FALSE(sm_0.isStart());
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}