#include <infuse_pom_crisp/KinematicChain.hpp>

using namespace std;
using namespace PositionManager;

typedef std::list<const PositionManager::Transform*> PosePtrList;

KinematicChain::KinematicChain(const FrameId& parent,  const FrameId& child) :
    _composed(parent, child, PositionManager::identityTransform()),
    _lastInsertedType(NONE)
{
}

void KinematicChain::update() const
{
    _composed._tr = identityTransform();
    for(list<const Transform*>::const_iterator it = _chain.begin(); it != _chain.end(); it++)
    {
        _composed._tr = _composed._tr*(*it)->transform;
    }
}

PositionManager::Pose KinematicChain::getPose() const
{
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

void KinematicChain::pushFixedTransform(const PositionManager::Transform& tr)
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
    
    //cout << "Added fixed transform :\n" << toString(tr) << endl;
}

void KinematicChain::pushFloatingTransform(const PositionManager::Transform* tr)
{
    _chain.push_back(tr);
    _lastInsertedType = FLOATING;

    //cout << "Added floating transform :\n" << toString(*tr) << endl;
}





