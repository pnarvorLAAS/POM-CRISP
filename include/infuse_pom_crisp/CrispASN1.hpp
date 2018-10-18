#ifndef DEF_POSITIONMANAGER_CRISPASN1_H
#define DEF_POSITIONMANAGER_CRISPASN1_H

#include <infuse_pom_crisp/Crisp.hpp>
#include <infuse_asn1_conversions/asn1_pom_conversions.hpp>

namespace PositionManager
{

class BitStreamedPose
{
    public:

    FrameId _parent;
    FrameId _child;
    mutable std::shared_ptr<asn1SccTransformWithCovariance> _asnPose;
    mutable BitStream _bstream;

    public:

    BitStreamedPose(const FrameId& parent = std::string("robot"),
                    const FrameId& child  = std::string("sensor"));
    BitStreamedPose(const BitStreamedPose& pose);
    ~BitStreamedPose();
    void encode(const PositionManager::Pose& pose);
    void decode() const;
    PositionManager::Pose getPose() const;
};

class CrispASN1 : public Crisp
{
    protected:

    public:

    CrispASN1();

    using Crisp::updatePose;
    using Crisp::getPose;
    using Crisp::getCachedPoses;

    int updatePose(const asn1SccTransformWithCovariance& pose); 
    int updatePose(BitStream bstream);
    
    int getPose(const FrameId& parent, const FrameId& child, BitStream& bstream) const;
    void getCachedPoses(std::vector<BitStreamedPose>& poses);
    int  getLatestCachedPoses(std::vector<BitStreamedPose>& poses);
};

};

#endif


