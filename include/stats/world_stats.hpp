
#pragma once 
#include <cstdint>

struct WorldStats {

    uint64_t steps=0;
    uint64_t bodyUpdates=0;
    uint64_t broadChecks=0;
    uint64_t narrowChecks=0;
    uint64_t contactsResolved=0;

    void resetStats(){
        steps=0;
        bodyUpdates=0; broadChecks=0; narrowChecks=0;contactsResolved=0;
    }

};
