#pragma once
#include "FullspaceSimulator.h"

namespace Lobo
{

class ModalWarpingModel;

class ModalWarpingSimulator : public FullspaceSimulator
{
public:
    ModalWarpingSimulator(Lobo::LoboDynamicScene *parent_scene);
    ~ModalWarpingSimulator();

    virtual void precompute();
    virtual void stepForward();

protected:
    ModalWarpingModel* modal_warping_model;
};
} // namespace Lobo