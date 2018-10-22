#include <infuse_pom_crisp/KinematicChain.hpp>

using namespace std;
using namespace PositionManager;

typedef std::list<const PositionManager::Transform*> PosePtrList;

KinematicChain::KinematicChain(const FrameId& parent,  const FrameId& child) :
    _composed(parent, child, PositionManager::identityTransform()),
    _lastInsertedType(NONE),
    _outdated(true)
{
    _composed._tr.time.microseconds = 10;
}

void KinematicChain::setOutdated() const
{
    _outdated = true;
}

void KinematicChain::update() const
{
    _composed._tr = identityTransform();

    for(list<const Transform*>::const_iterator it = _chain.begin(); it != _chain.end(); it++)
    {
        _composed._tr = _composed._tr*(*(*it));
    }

    _composed._parentTime = _composed._tr.time.microseconds;
    _composed._childTime = _composed._tr.time.microseconds;
    _outdated = false;
}

PositionManager::Pose KinematicChain::getPose() const
{
    if(_outdated)
        this->update();
    return _composed;
}

PositionManager::Pose KinematicChain::getPose(char& wasUpdated) const
{
    if(_outdated)
    {
        this->update();
        wasUpdated = 1;
    }
    else
    {
        wasUpdated = 0;
    }

    return _composed;
}

PositionManager::FrameId KinematicChain::getParent() const
{
    return _composed._parent;
}

PositionManager::FrameId KinematicChain::getChild() const
{
    return _composed._child;
}

void KinematicChain::pushTransform(const PositionManager::Transform& tr)
{
    if(_lastInsertedType != FIXED)
    {
        _cachedFixedTransform.push_back(tr);
        _chain.push_back(&(*(std::prev(_cachedFixedTransform.end()))));
        _lastInsertedType = FIXED;
    }
    else
    {
        *(std::prev(_cachedFixedTransform.end())) = (*(std::prev(_cachedFixedTransform.end())))*tr;
    }
}

void KinematicChain::pushTransform(const PositionManager::Transform* tr)
{
    _chain.push_back(tr);
    _lastInsertedType = FLOATING;
}





