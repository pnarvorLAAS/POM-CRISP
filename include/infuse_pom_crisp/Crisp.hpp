#ifndef DEF_POSITIONMANAGER_CRISP_H
#define DEF_POSITIONMANAGER_CRISP_H

#include <string>
#include <iostream>
#include <thread>
#include <mutex>
#include <map>
#include <vector>
#include <utility>
#include <sys/time.h>

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
    int getPose(const PositionManager::FrameId& parent,
                const PositionManager::FrameId& child, PositionManager::Pose& pose) const;
    int copyRobotGraph(PositionManager::Graph& dest);
    bool containsPose(const PositionManager::FrameId& parent,
                      const PositionManager::FrameId& child) const;

    // Members allocated to poses to export
    int getExportedPosesCount() const;
    std::vector<PositionManager::PoseId> getExportedPosesIds() const;
    bool getExportedPose(PositionManager::PoseId poseId, PositionManager::Pose& pose) const;
    int getExportedPoses(std::vector<PositionManager::Pose>& poses) const;

    bool addPoseToExport(const PositionManager::FrameId& parent,
                         const PositionManager::FrameId& child);
    bool removePoseFromExport(const PositionManager::FrameId& parent,
                              const PositionManager::FrameId& child);

    // Members dedicated to leaves (= ends of robot graph, usually sensors or actuators)
    int getLeafPose(const PositionManager::FrameId leaf, PositionManager::Pose& pose) const;
    int getLeavesCount() const;
    std::vector<PositionManager::FrameId> getLeavesFrameIds() const;
    bool addLeafToExport(const PositionManager::FrameId& leaf);
    bool removeLeafFromExport(const PositionManager::FrameId& leaf);
    void addLeavesToExport();

    PositionManager::FrameId getRobotBaseFrameId() const;

    std::map<PositionManager::PoseId,PositionManager::FrameIdPair> getMovableJoints() const;
    int getMovableJointsCount() const;

    //TO BE REMOVED NOT THREAD SAFE. DO NOT USE IF NOT NECESSARY
    std::shared_ptr<envire::core::EnvireGraph> getRobotGraph();

    protected:
    
    PositionManager::Graph _robotGraph;
    PositionManager::FrameId _robotBaseFrameId;
    std::vector<PositionManager::FrameId> _leaves;

    std::map<PositionManager::PoseId, PositionManager::FrameIdPair> _inputPoses;
    std::map<PositionManager::PoseId, PositionManager::FrameIdPair> _outputPoses;

    // grab and release mutex, wrapped for convenience
    void lockGraph() const; 
    void unlockGraph() const;
    mutable std::mutex _graphAccessMutex;
};

};

#endif




