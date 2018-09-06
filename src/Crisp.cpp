#include <infuse_pom_crisp/Crisp.hpp>

#include <fstream>

using namespace std;
using namespace PositionManager;

string getFrameIdPairString(const Pose& pose)
{
    return getFrameIdPairString(pose._parent, pose._child);
}

string getFrameIdPairString(const FrameId& parent, const FrameId& child)
{
    return parent + "->" + child;
}

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
    cout << to_string(_leaves.size()) << " leaves found : " << endl;
    int i = 0;
    for(list<FrameId>::iterator it = leavesList.begin(); it != leavesList.end(); it++)
    {
        _leaves[i] = *it;
        cout << _leaves[i] << endl;
        i++;
    }
    cout << "Done.\n" << endl;

    this->addLeavesToExport();

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

int Crisp::getPose(const FrameId& parent, const FrameId& child, Pose& pose) const
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

bool Crisp::containsPose(const FrameId& parent, const FrameId& child) const
{
    bool res = false;

    this->lockGraph();
    res = _robotGraph.containsEdge(parent, child);
    this->unlockGraph();

    return res;
}

int Crisp::copyRobotGraph(Graph& dest)
{
    this->lockGraph();
    dest.copyFrom(_robotGraph);
    this->unlockGraph();

    return 1;
}

int Crisp::getLeafPose(const FrameId leaf, Pose& pose) const
{
    return this->getPose(_robotBaseFrameId, leaf, pose);
}

int Crisp::getLeavesCount() const
{
    return _leaves.size();
}

vector<FrameId> Crisp::getLeavesFrameIds() const
{
    return _leaves;
}

bool Crisp::addLeafToExport(const FrameId& leaf)
{
    if(!this->containsPose(_robotBaseFrameId, leaf))
        return false;

    _exportedPoses.insert(std::pair<string, FrameIdPair>(leaf, FrameIdPair(_robotBaseFrameId, leaf)));

    return true;
}

bool Crisp::removeLeafFromExport(const FrameId& leaf)
{
    if(!_exportedPoses.erase(leaf))
        return false;
    return true;
}

void Crisp::addLeavesToExport()
{
    for(int i = 0; i < _leaves.size(); i++)
        this->addLeafToExport(_leaves[i]);
}

int Crisp::getExportedPosesCount() const
{
    return _exportedPoses.size();
}

vector<string> Crisp::getExportedPosesIds() const
{
    vector<FrameId> names(this->getExportedPosesCount());
    int i = 0;

    for(map<string, FrameIdPair>::const_iterator it = _exportedPoses.begin(); it != _exportedPoses.end(); ++it)
    {
        names[i] = it->first;
        i++;
    }

    return names;
}

int Crisp::getExportedPoses(vector<Pose>& poses) const
{
    int count = this->getExportedPosesCount();
    poses.resize(count);

    //cout << "Exported poses count : " << to_string(count) << endl;
    int i = 0;
    for(map<string, FrameIdPair>::const_iterator it = _exportedPoses.begin(); it != _exportedPoses.end(); ++it)
    {
        this->getPose(it->second.parent, it->second.child, poses[i]);
        i++;
    }

    return count;
}

bool Crisp::addPoseToExport(const FrameId& parent, const FrameId& child)
{
    if(!this->containsPose(parent, child))
        return false;

    _exportedPoses.insert(std::pair<string, FrameIdPair>(getFrameIdPairString(parent, child), FrameIdPair(parent, child)));

    return true;
}

bool Crisp::removePoseFromExport(const FrameId& parent, const FrameId& child)
{
    if(!_exportedPoses.erase(getFrameIdPairString(parent, child)))
        return false;
    return true;
}

FrameId Crisp::getRobotBaseFrameId() const
{
    return this->_robotBaseFrameId;
}

JointMap Crisp::getMovableJoints() const
{
    return _movableJoints;
}

int Crisp::getMovableJointsCount() const
{
    return _movableJoints.size();
}

// TO BE REMOVED NOT THREAD SAFE.
shared_ptr<envire::core::EnvireGraph> Crisp::getRobotGraph()
{
    return shared_ptr<envire::core::EnvireGraph>(&_robotGraph, [](envire::core::EnvireGraph*){});
}

inline void Crisp::lockGraph() const
{
    _graphAccessMutex.lock();
}

inline void Crisp::unlockGraph() const
{
    _graphAccessMutex.unlock();
}



