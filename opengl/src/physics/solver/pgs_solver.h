#pragma once

#include "physics_step_types.h"
#include <narrowphase/narrowphase_types.h>

class PGSSolver {
public:
    void init();
    void clear();

    void resolveContacts(
        const StepScope& scope, 
        ContactBatch& batch, 
        const int PGSiterations, 
        float dt
    );

    void postSolve(
        const StepScope& scope,
        ContactBatch& batch,
        RuntimeCaches& caches,
        int currentFrame,
        const float dt
    );
};