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

struct FrameIdPair
{
    FrameIdPair(FrameId p = FrameId("parent"), FrameId c = FrameId("child")) :
        parent(p), child(c) {}

    PositionManager::FrameId parent;
    PositionManager::FrameId child;
};

std::string getFrameIdPairString(const PositionManager::Pose& pose);
std::string getFrameIdPairString(const PositionManager::FrameId& parent, const PositionManager::FrameId& child);
    
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
    bool containsPose(const FrameId& parent, const FrameId& child);

    // Members allocated to poses to export
    int getExportedPosesCount();
    std::vector<std::string> getExportedPosesIds();
    int getExportedPoses(std::vector<PositionManager::Pose>& poses);
    bool addPoseToExport(const PositionManager::FrameId& parent, const PositionManager::FrameId& child);
    bool removePoseFromExport(const PositionManager::FrameId& parent, const PositionManager::FrameId& child);

    // Members dedicated to leaves (= ends of robot graph, usually sensors or actuators)
    int getLeafPose(const PositionManager::FrameId leaf, PositionManager::Pose& pose);
    int getLeavesCount();
    std::vector<PositionManager::FrameId> getLeavesFrameIds();
    bool addLeafToExport(const FrameId& leaf);
    bool removeLeafFromExport(const FrameId& leaf);
    void addLeavesToExport();

    PositionManager::FrameId getRobotBaseFrameId();

    JointMap& getMovableJoints();
    int getMovableJointsCount();

    //TO BE REMOVED NOT THREAD SAFE. DO NOT USE IF NOT NECESSARY
    std::shared_ptr<envire::core::EnvireGraph> getRobotGraph();

    protected:
    
    PositionManager::Graph _robotGraph;
    PositionManager::FrameId _robotBaseFrameId;
    std::vector<PositionManager::FrameId> _leaves;
    std::map<std::string, FrameIdPair> _exportedPoses;


    JointMap _movableJoints;

    // grab and release mutex, wrapped for convenience
    void lockGraph();
    void unlockGraph();
    std::mutex _graphAccessMutex;
};

};

#endif




