#include <iostream>
#include <Crisp.hpp>

using namespace std;
using namespace PositionManager;

int main(int argc, char** argv)
{
    if(argc < 2)
    {
        cout << "Please give an urdf file to read" << endl;
        return -1;
    }

    Crisp crisp;
    crisp.fromURDF(string(argv[1]));

    Transform tr;
    Pose pose1("FrontCamMountBase","FrontCamMountHead",tr); 
    Pose pose2("FrontCamMountBase","sgzgdbzsdbv",tr); // Error unknown FrameId
    Pose pose3("FrontCamMountBase","FrontCam",tr); // Error unknown edge, when both FrameIds valid but not linked directly together

    crisp.updateJointPose(pose1);
    crisp.updateJointPose(pose2);
    crisp.updateJointPose(pose3);

    return 0;
}


