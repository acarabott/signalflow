#pragma once

#include "signalflow/core/constants.h"
#include "signalflow/node/node.h"

namespace signalflow
{
class Smooth : public UnaryOpNode
{
public:
    Smooth(NodeRef input = nullptr, NodeRef smooth = 0.99);

    virtual void alloc() override;
    virtual void process(sample **out, int num_frames) override;

private:
    NodeRef smooth;
    std::vector<sample> values;
};

REGISTER(Smooth, "smooth")
}
