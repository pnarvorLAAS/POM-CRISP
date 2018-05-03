#include "accrisp_genom.h"

#include "crisp_genom_c_types.h"

#include "Crisp_genom.hpp"
#include <iostream>
using namespace std;

/* --- Task update_internal_tree ---------------------------------------- */


/** Codel uit_init of task update_internal_tree.
 *
 * Triggered by crisp_genom_start.
 * Yields to crisp_genom_waitloading.
 */
genom_event
uit_init(crisp_genom_ids *ids, const genom_context self)
{
    ids->isReady = false;
    ids->crisp = NULL;

    cout << "Waiting for input urdf file..." << endl;
    return crisp_genom_waitloading;
}


/** Codel uit_wait_loading of task update_internal_tree.
 *
 * Triggered by crisp_genom_waitloading.
 * Yields to crisp_genom_waitloading, crisp_genom_readjoints.
 */
genom_event
uit_wait_loading(const crisp_genom_ids *ids, const genom_context self)
{
    if(!ids->isReady)
    {
        usleep(500000);
        
        return crisp_genom_waitloading;
    }

    cout << "Crisp started !" << endl;
    return crisp_genom_readjoints;
}


/** Codel uit_read_joints_publications of task update_internal_tree.
 *
 * Triggered by crisp_genom_readjoints.
 * Yields to crisp_genom_pause_readjoints.
 */
genom_event
uit_read_joints_publications(const crisp_genom_jointPoseInput *jointPoseInput,
                             crisp_genom_ids *ids,
                             const genom_context self)
{
    BitStream bstream;
    asn1_bitstream* gbstream;

    for(int i = 0; i < ids->portInfos._length; i++)
    {
        if(jointPoseInput->read(ids->portInfos._buffer[i].name, self) != genom_ok)
        {
            continue;
        }

        gbstream = jointPoseInput->data(ids->portInfos._buffer[i].name, self);
        if(!gbstream)
        {
            continue;
        }

        //This section is to not read several time the same pose
        //if(gbstream->currentByte == -1)
        //    continue;
        //gbstream->currentByte = -1;
        if(gbstream->header.stamp.nsec == UINT32_MAX)
            continue;
        gbstream->header.stamp.nsec == UINT32_MAX;
        
        bstream.buf = gbstream->data._buffer;
        bstream.count = Pose_InFuse_REQUIRED_BYTES_FOR_ENCODING;
        bstream.currentByte = 0;
        bstream.currentBit = 0;

        ids->crisp->updateJointPose(bstream);
    }

    vector<PositionManager::Pose> poses;
    ids->crisp->getActiveLeavesPoses(poses);
    //cout << "Current Leaves States :" << endl;
    for(int i = 0; i < poses.size(); i++)
        cout << poses[i].toString() << "\n";
    if(poses.size() != 0)
        cout << endl;
    

    return crisp_genom_pause_readjoints;
}


/** Codel uit_stop of task update_internal_tree.
 *
 * Triggered by crisp_genom_stop.
 * Yields to crisp_genom_ether.
 */
genom_event
uit_stop(crisp_genom_ids *ids, const genom_context self)
{
    delete ids->crisp;
    cout << "Crisp stopped." << endl;
    return crisp_genom_ether;
}
