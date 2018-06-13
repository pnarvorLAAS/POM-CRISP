#ifndef DEF_POSITIONMANAGER_CRISPASN1_H
#define DEF_POSITIONMANAGER_CRISPASN1_H

#include "Crisp.hpp"

#include <conversions/asn1_pom_conversions.hpp>

namespace PositionManager
{

class CrispASN1 : public Crisp
{
    public:

    CrispASN1();

    using Crisp::updateJointPose;
    using Crisp::getLeafPose;
    int updateJointPose(const Pose_InFuse& pose); 
    int updateJointPose(BitStream bstream);

    int getLeafPose(const PositionManager::FrameId frameId, BitStream& bstream);
};

};

#endif


