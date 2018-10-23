#include <infuse_pom_crisp/CrispASN1.hpp>

using namespace std;
using namespace PositionManager;


BitStreamedPose::BitStreamedPose(const FrameId& parent, const FrameId& child) :
    _parent(parent),
    _child(child),
    _asnPose(new asn1SccTransformWithCovariance)
{
    BitStream_Init(&_bstream, new uint8_t[asn1SccTransformWithCovariance_REQUIRED_BYTES_FOR_ENCODING], asn1SccTransformWithCovariance_REQUIRED_BYTES_FOR_ENCODING);
}

BitStreamedPose::BitStreamedPose(const BitStreamedPose& pose) :
    _parent(pose._parent),
    _child(pose._child),
    _asnPose(new asn1SccTransformWithCovariance)
{
    memcpy(_asnPose.get(), pose._asnPose.get(), asn1SccTransformWithCovariance_REQUIRED_BYTES_FOR_ENCODING);
    BitStream_Init(&_bstream, new uint8_t[asn1SccTransformWithCovariance_REQUIRED_BYTES_FOR_ENCODING], asn1SccTransformWithCovariance_REQUIRED_BYTES_FOR_ENCODING);
    memcpy(_bstream.buf, pose._bstream.buf, asn1SccTransformWithCovariance_REQUIRED_BYTES_FOR_ENCODING);
}

BitStreamedPose::~BitStreamedPose()
{
    delete[] _bstream.buf;
}

void BitStreamedPose::encode(const PositionManager::Pose& pose)
{
    int errorCode;
    flag res;

    toASN1SCC(pose, *_asnPose);

    toASN1SCC(string("CRISP"), _asnPose->metadata.producerId);

    _asnPose->metadata.dataEstimated.arr[0] = 1;
    _asnPose->metadata.dataEstimated.arr[1] = 1;
    _asnPose->metadata.dataEstimated.arr[2] = 1;
    _asnPose->metadata.dataEstimated.arr[3] = 1;
    _asnPose->metadata.dataEstimated.arr[4] = 1;
    _asnPose->metadata.dataEstimated.arr[5] = 1;
    _asnPose->metadata.dataEstimated.arr[6] = 1;

    //_bstream.currentByte = 0;
    //_bstream.currentBit = 0;
    BitStream_Init(&_bstream, _bstream.buf, asn1SccTransformWithCovariance_REQUIRED_BYTES_FOR_ENCODING);
    res = asn1SccTransformWithCovariance_Encode(_asnPose.get(), &_bstream, &errorCode, TRUE);
    if(!res)
        std::cout << "error, asn1SccTransformWithCovariance encoding error : " << errorCode << std::endl;

    _parent = pose._parent;
    _child = pose._child;
}

void BitStreamedPose::decode() const
{
    flag res;
    int errorCode;

    _bstream.currentByte = 0;
    _bstream.currentBit = 0;
    res = asn1SccTransformWithCovariance_Decode(_asnPose.get(), &_bstream, &errorCode);
    if(!res)
        std::cout << "error, asn1SccTransformWithCovariance decoding error : " << errorCode << std::endl;
}

PositionManager::Pose BitStreamedPose::getPose() const
{
    Pose res;

    this->decode();
    fromASN1SCC(*_asnPose, res);

    return res;
}

CrispASN1::CrispASN1()
{
}

int CrispASN1::updatePose(const asn1SccTransformWithCovariance& asnPose)
{
    PositionManager::Pose pose;
   
    fromASN1SCC(asnPose, pose);

    //cout << "Got pose :" << endl << pose.toStringVerbose() << endl << endl;
    return this->updatePose(pose);
}

int CrispASN1::updatePose(BitStream bstream)
{
    asn1SccTransformWithCovariance asnPose;
    flag res;
    int errorCode;

    bstream.currentByte = 0;
    bstream.currentBit = 0;
    res = asn1SccTransformWithCovariance_Decode(&asnPose, &bstream, &errorCode);
    if(!res)
        cout << "error CrispASN1 : decoding error : " << errorCode << endl;

    return this->updatePose(asnPose);
}

int CrispASN1::getPose(const PositionManager::FrameId& parent, const PositionManager::FrameId& child, BitStream& bstream) const
{
    PositionManager::Pose pose;
    asn1SccTransformWithCovariance asnPose;
    int errorCode;
    flag res;

    if(!this->getPose(parent, child, pose))
        return 0;
    toASN1SCC(pose, asnPose);
    asnPose.metadata.dataEstimated.arr[0] = 1;
    asnPose.metadata.dataEstimated.arr[1] = 1;
    asnPose.metadata.dataEstimated.arr[2] = 1;
    asnPose.metadata.dataEstimated.arr[3] = 1;
    asnPose.metadata.dataEstimated.arr[4] = 1;
    asnPose.metadata.dataEstimated.arr[5] = 1;
    asnPose.metadata.dataEstimated.arr[6] = 1;

    BitStream_Init(&bstream, bstream.buf, asn1SccTransformWithCovariance_REQUIRED_BYTES_FOR_ENCODING);
    res = asn1SccTransformWithCovariance_Encode(&asnPose, &bstream, &errorCode, TRUE);
    if(!res)
    {
        cout << "error CrispASN1 : encoding error : " << errorCode << endl;
        return 0;
    }

    return 1;
}

int CrispASN1::getCachedPoses(std::vector<BitStreamedPose>& poses)
{
    static vector<Pose> posesTmp;
    static vector<char> wasUpdated;

    int nbPoses = this->getCachedPoses(posesTmp, wasUpdated);

    if(poses.size() < nbPoses)
        poses.resize(nbPoses);

    //cout << "Updated pose :\n" << posesTmp[2].toStringVerbose() << endl;
    for(int i = 0; i < nbPoses; i++)
    {
        // Have to reencode each loop because order of poses not garanted
        //cout << "Updated pose :\n" << posesTmp[i].toStringVerbose() << endl;
        poses[i].encode(posesTmp[i]);
    }

    //cout << "Getting all poses : " << poses.size() << endl;

    return poses.size();
}

int CrispASN1::getLatestCachedPoses(std::vector<BitStreamedPose>& poses)
{
    static vector<Pose> posesTmp;
    static vector<char> wasUpdated;
    int nbPoses = 0;

    this->getCachedPoses(posesTmp, wasUpdated);
    for(int i = 0; i < wasUpdated.size(); i++)
    {
        if(wasUpdated[i] != 0)
            nbPoses++;
    }

    if(poses.size() < nbPoses)
        poses.resize(nbPoses);

    int j = 0;
    for(int i = 0; i < wasUpdated.size(); i++)
    {
        if(wasUpdated[i] != 0)
        {
            //cout << "Updated pose latest :\n" << posesTmp[i].toStringVerbose() << endl;
            poses[j].encode(posesTmp[i]);
            j++;
        }
    }

    //for(int j = 0; j < nbPoses; j++)
    //{
    //    cout << "New pose :\n" << poses[j]._child << "->" << poses[j]._parent << endl;
    //}
    //cout << "\n" << endl;

    //cout << "Getting new poses : " << nbPoses << endl;

    return nbPoses;
}
