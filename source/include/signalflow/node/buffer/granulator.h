#pragma once

#include "signalflow/buffer/buffer.h"
#include "signalflow/node/node.h"

#include <vector>

namespace signalflow
{
class Grain
{
public:
    Grain(BufferRef buffer, int start, int length, float rate, float pan);

    bool finished();

    BufferRef buffer;
    double sample_start;
    int sample_length;
    double samples_done;
    float rate;
    float pan;
};

class Granulator : public Node
{
public:
    Granulator(BufferRef buffer = nullptr,
               NodeRef clock = 0,
               NodeRef pos = 0,
               NodeRef duration = 0.1,
               NodeRef pan = 0.0,
               NodeRef rate = 1.0,
               NodeRef max_grains = 2048);

    BufferRef buffer;
    BufferRef envelope;

    NodeRef pos;
    NodeRef clock;
    NodeRef duration;
    NodeRef pan;
    NodeRef rate;
    NodeRef max_grains;

    virtual void process(Buffer &out, int num_frames);

private:
    sample clock_last;

    std::vector<Grain *> grains;
};

REGISTER(Granulator, "granulator")
}
