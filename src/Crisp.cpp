#include "Crisp.hpp"

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
    UrdfParser parser;

    cout << "Parsing URDF " << urdfFilename << endl;
    if(!parser.parseURDF(urdfFilename))
    {
        cout << "Error Crisp : parsing failed" << endl;
        return 0;
    }
    cout << "Parsing susccessfull" << endl;

    _movableJoints = parser._movableJoints;
    
    lockGraph();
    
    _robotBaseFrameId = parser._rootFrameId;
    cout << "Root node is " << _robotBaseFrameId << endl;

    for(list<FrameId>::iterator it = parser._frameIds.begin(); it != parser._frameIds.end(); ++it)
    {
        _robotGraph.addFrame(*it);
    }
    for(list<Pose>::iterator it = parser._poses.begin(); it != parser._poses.end(); ++it)
    {
        _robotGraph.addTransform(it->_parent, it->_child, it->_tr);
    }

    cout << "Checking leaf nodes... ";
    list<FrameId> leavesList = _robotGraph.getLeaves(&this->_robotBaseFrameId);
    _leaves.resize(leavesList.size());
    _leavesActiveState.resize(_leaves.size());
    cout << to_string(_leaves.size()) << " leaves found : " << endl;
    int i = 0;
    for(list<FrameId>::iterator it = leavesList.begin(); it != leavesList.end(); it++)
    {
        _leaves[i] = *it;
        //_leavesActiveState[i] = true;
        _leavesActiveState[i] = false;
        cout << _leaves[i] << endl;
        i++;
    }
    cout << "Done.\n" << endl;

    unlockGraph();
    return 1;
}

int Crisp::updateJointPose(const Pose& pose)
{
    // TODO Handle Generic Exception
    this->lockGraph();
    try
    {
        _robotGraph.updateTransform(pose._parent, pose._child, pose._tr);
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

int Crisp::getPose(const FrameId& parent, const FrameId& child, Pose& pose)
{
    TimeUs tval;
    // TODO Handle generic Exception
    this->lockGraph();
    try
    {
        pose._tr = _robotGraph.getTransform(_robotBaseFrameId, child);
    }
    catch(exception& e)
    {
        cout << "Caught exception, Crisp updateJointPose : " << e.what() << endl;
        this->unlockGraph();
        return 0;
    }

    tval = TimeManager::now();
    this->unlockGraph();

    pose._parent = parent;
    pose._parentTime = tval;
    pose._child = child;
    pose._childTime = tval;

    return 1;
}

int Crisp::copyRobotGraph(Graph& dest)
{
    this->lockGraph();
    dest.copyFrom(_robotGraph);
    this->unlockGraph();

    return 1;
}

int Crisp::getLeafPose(const FrameId leaf, Pose& pose)
{
    return this->getPose(_robotBaseFrameId, leaf, pose);
}

int Crisp::getLeavesCount()
{
    return _leaves.size();
}

vector<FrameId> Crisp::getLeavesNames()
{
    return _leaves;
}

int Crisp::getActiveLeavesCount()
{
    int count = 0;
    for(unsigned int i = 0; i < _leavesActiveState.size(); i++)
    {
        if(_leavesActiveState[i])
            count++;
    }

    return count;
}

vector<FrameId> Crisp::getActiveLeavesNames()
{
    vector<FrameId> names(this->getActiveLeavesCount());
    int j = 0;

    for(unsigned int i = 0; i < _leaves.size(); i++)
    {
        if(_leavesActiveState[i])
        {
            names[j] = _leaves[i];
            j++;
        }
    }

    return names;
}

int Crisp::getActiveLeavesPoses(std::map<FrameId, Pose>& poses)
{
    int count = this->getActiveLeavesCount();
    Pose pTmp;
    poses.clear();

    //cout << "ActiveLeaves count : " << to_string(count) << endl;
    for(unsigned int i = 0; i < _leaves.size(); i++)
    {
        if(_leavesActiveState[i])
        {
            if(!this->getLeafPose(_leaves[i], pTmp))
                continue;
            poses.insert(pair<FrameId, Pose>(_leaves[i], pTmp));
        }
    }

    return count;
}

int Crisp::toggleLeafState(PositionManager::FrameId leaf)
{
    for(unsigned int i = 0; i < _leavesActiveState.size(); i++)
    {
        if(_leaves[i] == leaf)
        {
            if(_leavesActiveState[i])
                _leavesActiveState[i] = false;
            else
                _leavesActiveState[i] = true;

            return 1;
        }
    }

    cout << "Error, Crisp : Leaf \" " << leaf << "\" does not exist" << endl;
    return 0;
}

FrameId Crisp::getRobotBaseFrameId()
{
    return this->_robotBaseFrameId;
}

JointMap& Crisp::getMovableJoints()
{
    return _movableJoints;
}

int Crisp::getMovableJointsCount()
{
    return _movableJoints.size();
}

// TO BE REMOVED NOT THREAD SAFE.
shared_ptr<envire::core::EnvireGraph> Crisp::getRobotGraph()
{
    return shared_ptr<envire::core::EnvireGraph>(&_robotGraph, [](envire::core::EnvireGraph*){});
}

inline void Crisp::lockGraph()
{
    _graphAccessMutex.lock();
}

inline void Crisp::unlockGraph()
{
    _graphAccessMutex.unlock();
}



