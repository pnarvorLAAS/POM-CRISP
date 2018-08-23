#include <infuse_pom_crisp/CrispASN1.hpp>

using namespace std;
using namespace PositionManager;

CrispASN1::CrispASN1()
{}

int CrispASN1::updateJointPose(const asn1SccTransformWithCovariance& asnPose)
{
    PositionManager::Pose pose;
   
    fromASN1SCC(asnPose, pose);

    //cout << "Got pose :" << endl << pose.toStringVerbose() << endl << endl;

    return this->updateJointPose(pose);
}

int CrispASN1::updateJointPose(BitStream bstream)
{
    asn1SccTransformWithCovariance asnPose;
    flag res;
    int errorCode;

    bstream.currentByte = 0;
    bstream.currentBit = 0;
    res = asn1SccTransformWithCovariance_Decode(&asnPose, &bstream, &errorCode);
    if(!res)
    {
        cout << "error CrispASN1 : decoding error : " << errorCode << endl;
    }

    return this->updateJointPose(asnPose);
}

int CrispASN1::getLeafPose(const PositionManager::FrameId frameId, BitStream& bstream)
{
    PositionManager::Pose pose;
    asn1SccTransformWithCovariance asnPose;
    int errorCode;
    flag res;

    if(!this->getLeafPose(frameId, pose))
        return 0;
    toASN1SCC(pose, asnPose);

    BitStream_Init(&bstream, bstream.buf, asn1SccTransformWithCovariance_REQUIRED_BYTES_FOR_ENCODING);
    res = asn1SccTransformWithCovariance_Encode(&asnPose, &bstream, &errorCode, TRUE);
    if(!res)
    {
        cout << "error CrispASN1 : encoding error : " << errorCode << endl;
        return 0;
    }

    return 1;
}

