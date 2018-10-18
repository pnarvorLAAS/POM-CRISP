#ifndef DEF_POSITIONMANAGER_CRISP_H
#define DEF_POSITIONMANAGER_CRISP_H

#include <string>
#include <iostream>
#include <thread>
#include <mutex>
#include <map>
#include <vector>
#include <utility>
#include <algorithm>
#include <sys/time.h>

#include <infuse_pom_base/PositionManagerBase.hpp>
#include <infuse_pom_base/UrdfParser.hpp>

#include "KinematicChain.hpp"

namespace PositionManager
{

typedef Graph::edge_descriptor EdgeDescriptor;

class Crisp
{
    typedef std::list<KinematicChain> ChainList;
    typedef std::list<const KinematicChain*> ChainPtrList;
    typedef std::map<EdgeDescriptor, std::pair<PositionManager::Transform, ChainPtrList>> MovablePoseRegister;
    typedef std::pair<EdgeDescriptor, std::pair<PositionManager::Transform, ChainPtrList>> MovablePoseRegisterEl;

    protected:

    PositionManager::Graph _robotGraph;
    PositionManager::FrameId _robotBaseFrameId;

    MovablePoseRegister _inputPoses;
    ChainList _cachedPoses;

    // grab and release mutex, wrapped for convenience
    void lockGraph() const; 
    void unlockGraph() const;
    mutable std::mutex _graphAccessMutex;

    bool isCached(const FrameId& parent, const FrameId& child);
    ChainList::iterator findCached(const FrameId& parent, const FrameId& child);
    void addPoseToCache(const FrameId& parent, const FrameId& child);
    void removePoseFromCache(const FrameId& parent, const FrameId& child);

    public:

    Crisp();
    ~Crisp();

    // Only member function able to directly write/read in the graph transforms. (lock the mutex)
    int fromURDF(std::string filename); 
    int updatePose(const PositionManager::Pose& pose);
    int getPose(const PositionManager::FrameId& parent,
                const PositionManager::FrameId& child, PositionManager::Pose& pose) const;
    int copyRobotGraph(PositionManager::Graph& dest);
    bool containsPose(const PositionManager::FrameId& parent,
                      const PositionManager::FrameId& child) const;

    bool isMovable(const FrameId& parent, const FrameId& child);
    void getCachedPoses(std::vector<PositionManager::Pose>& poses);

    ////TO BE REMOVED NOT THREAD SAFE. DO NOT USE IF NOT NECESSARY
    std::shared_ptr<envire::core::EnvireGraph> getRobotGraph();
};

};

#endif




