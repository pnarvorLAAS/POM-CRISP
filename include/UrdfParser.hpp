#ifndef DEF_POSITIONMANAGER_URDFPARSER_H
#define DEF_POSITIONMANAGER_URDFPARSER_H

#include <string>
#include <iostream>
#include <thread>
#include <mutex>

#include <PositionManagerBase.hpp>
#include <urdf_parser/urdf_parser.h>

namespace PositionManager
{

typedef std::pair<PositionManager::FrameId, PositionManager::FrameId> Joint;
typedef std::map<std::string, Joint> JointMap;

class UrdfParser
{
    public:

    UrdfParser();

    void reset();
    int parseURDF(std::string path);

    public:

    PositionManager::FrameId _rootFrameId;
    std::list<PositionManager::FrameId> _frameIds;
    std::list<PositionManager::Pose> _poses;

    JointMap _movableJoints;

    protected:

    int extractCovariances(const std::string& xmlStr, std::map<std::string,PositionManager::Covariance>& jointCovariances);
    int parseCovarianceString(const char* strIn, PositionManager::Covariance& cov);
};

};

#endif


