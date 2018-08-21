#ifndef DEF_POSITIONMANAGER_CRISPASN1_H
#define DEF_POSITIONMANAGER_CRISPASN1_H

#include <infuse_pom_crisp/Crisp.hpp>
#include <infuse_asn1_conversions/asn1_pom_conversions.hpp>

namespace PositionManager
{

class CrispASN1 : public Crisp
{
    public:

    CrispASN1();

    using Crisp::updateJointPose;
    using Crisp::getLeafPose;
    int updateJointPose(const asn1SccTransformWithCovariance& pose); 
    int updateJointPose(BitStream bstream);

    int getLeafPose(const PositionManager::FrameId frameId, BitStream& bstream);
};

};

#endif


