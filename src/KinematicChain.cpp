#include <infuse_pom_crisp/KinematicChain.hpp>

using namespace std;
using namespace PositionManager;

typedef std::list<const PositionManager::Transform*> PosePtrList;

KinematicChain::KinematicChain(const FrameId& parent,  const FrameId& child) :
    PosePtrList(),
    _composed(parent, child, PositionManager::identityTransform())
{
}

void KinematicChain::update() const
{
    _composed._tr = identityTransform();
    for(list<const Transform*>::const_iterator it = this->begin(); it != this->end(); it++)
    {
        _composed._tr = _composed._tr*(*it)->transform;
    }
}

PositionManager::Pose KinematicChain::getTransform() const
{
    return _composed;
}


