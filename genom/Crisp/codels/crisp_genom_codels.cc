#include "accrisp_genom.h"

#include "crisp_genom_c_types.h"

#include "Crisp_genom.hpp"
#include <iostream>
using namespace std;

/* --- Function load_from_file ------------------------------------------ */

/** Codel load_from_file of function load_from_file.
 *
 * Returns genom_ok.
 * Throws mrism_genom_e_sys.
 */
genom_event
load_from_file(const char path[512], crisp_genom_ids *ids,
               const genom_context self)
{
    ids->crisp = new Crisp_genom();
    if(!ids->crisp->fromURDF(path))
    {
        cout << "Unable to load file : " << path << endl;
        return genom_ok;
    }

    PositionManager::JointMap& jmap = ids->crisp->getMovableJoints();
    int numberOfPorts = jmap.size();
    cout << "Number of ports to create : " << to_string(numberOfPorts) << endl;
    
    genom_sequence_reserve(&ids->portInfos, numberOfPorts);
    ids->portInfos._length = ids->portInfos._maximum;

    int i = 0;
    for(PositionManager::JointMap::iterator it = jmap.begin(); it != jmap.end(); ++it)
    {
        cout << "Added joint port : " << it->first << endl;
        strncpy(ids->portInfos._buffer[i].name, it->first.c_str(), sizeof(ids->portInfos._buffer[i].name));
        ids->portInfos._buffer[i].last.sec = 0;
        ids->portInfos._buffer[i].last.nsec = 0;

        i++;
    }

    ids->isReady = true;
    return genom_ok;
}


/* --- Function toggle_leaf --------------------------------------------- */

/** Codel toggle_leaf of function toggle_leaf.
 *
 * Returns genom_ok.
 */
genom_event
toggle_leaf(const char leaf[512], crisp_genom_ids *ids,
            const genom_context self)
{
    ids->crisp->toggleLeafState(string(leaf));
    return genom_ok;
}
