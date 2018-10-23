#include <infuse_pom_crisp/Crisp.hpp>

#include <fstream>

using namespace std;
using namespace PositionManager;

Crisp::Crisp()
{
    _robotBaseFrameId = "RobotBase";
}

Crisp::~Crisp()
{
}

int Crisp::fromURDF(string urdfFilename)
{
    int64_t timeUs = TimeManager::now();
    cout << "Time : " << timeUs << endl;

    UrdfParser parser;
    cout << "Parsing URDF " << urdfFilename << endl;
    if(!parser.parseURDF(urdfFilename))
    {
        cout << "Error Crisp : parsing failed" << endl;
        return 0;
    }
    cout << "Parsing susccessfull" << endl;

    this->lockGraph();
    _robotBaseFrameId = parser._rootFrameId;
    cout << "Root node is " << _robotBaseFrameId << endl;

    for(list<FrameId>::iterator it = parser._frameIds.begin(); it != parser._frameIds.end(); ++it)
    {
        _robotGraph.addFrame(*it);
    }
    for(list<Pose>::iterator it = parser._poses.begin(); it != parser._poses.end(); ++it)
    {
        it->_tr.time.microseconds = timeUs;
        _robotGraph.addTransform(it->_parent, it->_child, it->_tr);
    }

    EdgeDescriptor edge;
    Pose p;
    for(list<FrameIdPair>::const_iterator it = parser._movableJoints.begin(); it != parser._movableJoints.end(); it++)
    {
        try
        {
            p._parent = it->parent;
            p._parentTime = timeUs;
            p._child = it->child;
            p._childTime = timeUs;
            p._tr = _robotGraph.getTransform(it->parent, it->child);

            //cout << "Inserting inputPose :\n" << p.toStringVerbose() << endl;
           
            edge = _robotGraph.getEdge(it->parent, it->child);
            _inputPoses.insert(MovablePoseRegisterEl(edge, pair<Pose, ChainPtrList>(p, ChainPtrList())));
        }
        catch(exception& e)
        {
            cout << "Caught exception, Crisp::fromURDF : " << e.what() << endl;
            this->unlockGraph();
            return 0;
        }
    }

    cout << "Checking leaf nodes... ";
    list<FrameId> leavesList = _robotGraph.getLeaves(&this->_robotBaseFrameId);
    for(list<FrameId>::iterator it = leavesList.begin(); it != leavesList.end(); it++)
        this->addPoseToCache(_robotBaseFrameId, *it); 
    cout << "Done.\n" << endl;

    this->unlockGraph();

    return 1;
}

int Crisp::updatePose(const Pose& pose)
{
    // TODO Handle Generic Exception
    this->lockGraph();
    try
    {
        EdgeDescriptor edge = _robotGraph.getEdge(pose._parent, pose._child);
        MovablePoseRegister::iterator it = _inputPoses.find(edge);
        if(it == _inputPoses.end())
        {
            cout << pose._child << "->" << pose._parent << " is not registered as a movable pose : the attribute type in the .urdf is not set to \"floating\"" << endl;
            return 0;
        }
        
        _robotGraph.updateTransform(pose._parent, pose._child, pose._tr);
        this->updateCache(edge, pose);
    }
    catch(exception& e)
    {
        cout << "Caught exception, Crisp updateJointPose : " << e.what() << endl;
        this->unlockGraph();
        return 0;
    }
    this->unlockGraph();
   
    return 1;
}

int Crisp::getPose(const FrameId& parent, const FrameId& child, Pose& pose) const
{
    TimeUs tval;
    // TODO Handle generic Exception
    this->lockGraph();
    try
    {
        pose._tr = _robotGraph.getTransform(parent, child);
    }
    catch(exception& e)
    {
        cout << "Caught exception, Crisp updateJointPose : " << e.what() << endl;
        this->unlockGraph();
        return 0;
    }

    this->unlockGraph();

    return 1;
}

bool Crisp::containsPose(const FrameId& parent, const FrameId& child) const
{
    bool res = true;

    this->lockGraph();
    try
    {
       _robotGraph.getTransform(parent, child);
    }
    catch(exception& e)
    {
        res = false;
    }
    this->unlockGraph();

    return res;
}

bool Crisp::isMovable(const FrameId& parent, const FrameId& child)
{
    if(_inputPoses.find(_robotGraph.getEdge(parent,child)) == _inputPoses.end())
        return false;
    return true;
}

int Crisp::getMovableJoints(std::vector<FrameIdPair>& poses)
{
    if(poses.size() < _inputPoses.size())
        poses.resize(_inputPoses.size());
    
    MovablePoseRegister::iterator it = _inputPoses.begin();
    for(int i = 0; i < _inputPoses.size(); i++)
    {
        if(it == _inputPoses.end())
            throw runtime_error("Crisp::getMovableJoints : fatal error");
        poses[i] = FrameIdPair(it->second.first._parent, it->second.first._child);
    }

    return _inputPoses.size();
}

int Crisp::copyRobotGraph(Graph& dest)
{
    this->lockGraph();
    dest.copyFrom(_robotGraph);
    this->unlockGraph();

    return 1;
}


bool Crisp::isCached(const FrameId& parent, const FrameId& child)
{
    return this->findCached(parent,child) != _cachedPoses.end();
}

Crisp::ChainList::iterator Crisp::findCached(const FrameId& parent, const FrameId& child)
{
    ChainList::iterator it;
    for(it = _cachedPoses.begin(); it != _cachedPoses.end(); it++)
    {
        if(it->getParent() == parent && it->getChild() == child)
            return it;
    }

    return it;
}

bool Crisp::addPoseToCache(const FrameId& parent, const FrameId& child)
{
    if(isCached(parent,child) || !containsPose(parent, child))
        return false;

    cout << "Adding " << child << "->" << parent << " to cache" << endl;

    _cachedPoses.push_back(KinematicChain(parent, child));
    ChainList::iterator chain = std::prev(_cachedPoses.end());

    Path path = *_robotGraph.getPath(parent, child, false);
    for(int i = 0; i < path.getSize() - 1; i++)
    {
        EdgeDescriptor edge = _robotGraph.getEdge(path[i],path[i+1]);
        MovablePoseRegister::iterator it = _inputPoses.find(edge);
        if(it == _inputPoses.end())
        {
            chain->pushTransform(_robotGraph.getTransform(path[i], path[i+1]));
        }
        else
        {
            chain->pushTransform(&(it->second.first._tr));
            it->second.second.push_back(&(*chain));
        }
    }
    chain->update();

    return true;
}

void Crisp::removePoseFromCache(const FrameId& parent, const FrameId& child)
{
    ChainList::iterator chain = this->findCached(parent,child);
    if(chain != _cachedPoses.end())
    {
        for(MovablePoseRegister::iterator it = _inputPoses.begin(); it != _inputPoses.end(); it++)
        {
            ChainPtrList::iterator chain2 = std::find(it->second.second.begin(), it->second.second.end(), &(*chain));
            if(chain2 != it->second.second.end())
                it->second.second.erase(chain2);
        }
        _cachedPoses.erase(chain);
    }
}

int Crisp::updateCache(const EdgeDescriptor& edge, const Pose& pose)
{
    try
    {
        pair<Pose, ChainPtrList>* it1 = &_inputPoses.at(edge);
        it1->first._tr = pose._tr;
        for(ChainPtrList::iterator it2 = it1->second.begin(); it2 != it1->second.end(); it2++)
        {
            //cout << "Setting outdated : " << (*it2)->getChild() << "->" << (*it2)->getParent() << endl;
            (*it2)->setOutdated();
        }
    }
    catch(exception& e)
    {
        return 0;
    }
    return 1;
}

int Crisp::getCachedPoses(std::vector<PositionManager::Pose>& poses)
{
    if(poses.size() < _cachedPoses.size())
        poses.resize(_cachedPoses.size());

    ChainList::iterator it = _cachedPoses.begin();
    for(int i = 0; i < _cachedPoses.size(); i++)
    {
        if(it == _cachedPoses.end())
            throw runtime_error("Crisp::getCachedPoses : fatal error");
        poses[i] = it->getPose();
        it = std::next(it);
    }

    return _cachedPoses.size();
}

int Crisp::getCachedPoses(std::vector<PositionManager::Pose>& poses, vector<char>& wasUpdated)
{
    if(poses.size() < _cachedPoses.size())
        poses.resize(_cachedPoses.size());
    if(wasUpdated.size() < _cachedPoses.size())
        wasUpdated.resize(_cachedPoses.size());

    ChainList::iterator it = _cachedPoses.begin();
    for(int i = 0; i < _cachedPoses.size(); i++)
    {
        if(it == _cachedPoses.end())
            throw runtime_error("Crisp::getCachedPoses : fatal error");
        poses[i] = it->getPose(wasUpdated[i]);
        it = std::next(it);
    }

    return _cachedPoses.size();
}

// TO BE REMOVED NOT THREAD SAFE.
shared_ptr<envire::core::EnvireGraph> Crisp::getRobotGraph()
{
    return shared_ptr<envire::core::EnvireGraph>(&_robotGraph, [](envire::core::EnvireGraph*){});
}

inline void Crisp::lockGraph() const
{
    //_graphAccessMutex.lock();
}

inline void Crisp::unlockGraph() const
{
    //_graphAccessMutex.unlock();
}



