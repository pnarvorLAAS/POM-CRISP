#ifndef _DEF_POSITION_MANAGER_KINEMATIC_CHAIN_H_
#define _DEF_POSITION_MANAGER_KINEMATIC_CHAIN_H_

#include <iostream>
#include <list>

#include <infuse_pom_base/PositionManagerBase.hpp>

namespace PositionManager
{

class KinematicChain : public std::list<const PositionManager::Transform*>
{
    protected:

    mutable PositionManager::Pose _composed;

    public:

    KinematicChain(const PositionManager::FrameId& parent, const PositionManager::FrameId& child);

    void update() const;
    PositionManager::Pose getTransform() const;
};

};

#endif //_DEF_POSITION_MANAGER_KINEMATIC_CHAIN_H_
