#ifndef DEF_POSITIONMANAGER_CRISP_H
#define DEF_POSITIONMANAGER_CRISP_H

#include <string>
#include <iostream>
#include <thread>
#include <mutex>
#include <map>
#include <vector>
#include <utility>

#include <infuse_pom_base/PositionManagerBase.hpp>
#include <infuse_pom_base/UrdfParser.hpp>

namespace PositionManager
{
    

class Crisp
{
    public:

    Crisp();
    ~Crisp();

    // Only member function able to directly write/read in the graph transforms. (lock the mutex)
    int fromURDF(std::string filename); 
    int updateJointPose(const PositionManager::Pose& pose);
    int getPose(const PositionManager::FrameId& parent, const PositionManager::FrameId& child, PositionManager::Pose& pose);
    int copyRobotGraph(PositionManager::Graph& dest);

    // Members dedicated to leaves (= ends of robot graph, usually sensors or actuators)
    int getLeafPose(const PositionManager::FrameId leaf, PositionManager::Pose& pose);
    int getLeavesCount();
    std::vector<PositionManager::FrameId> getLeavesNames();
    int getActiveLeavesCount();
    std::vector<PositionManager::FrameId> getActiveLeavesNames();
    int getActiveLeavesPoses(std::map<PositionManager::FrameId, PositionManager::Pose>& poses);
    int toggleLeafState(PositionManager::FrameId leaf);
    PositionManager::FrameId getRobotBaseFrameId();

    //TO BE REMOVED NOT THREAD SAFE. DO NOT USE IF NOT NECESSARY
    std::shared_ptr<envire::core::EnvireGraph> getRobotGraph();
    JointMap& getMovableJoints();
    int getMovableJointsCount();
    
    protected:
    
    PositionManager::Graph _robotGraph;
    
    std::vector<PositionManager::FrameId> _leaves;
    std::vector<bool> _leavesActiveState;
    PositionManager::FrameId _robotBaseFrameId;

    JointMap _movableJoints;

    // grab and release mutex, wrapped for convenience
    void lockGraph();
    void unlockGraph();
    std::mutex _graphAccessMutex;
};

};

#endif




