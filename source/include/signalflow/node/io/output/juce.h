#pragma once

#ifdef JUCE_MODULE_AVAILABLE_juce_audio_basics

#define AudioOut AudioOut_JUCE

#include "signalflow/core/graph.h"
#include "signalflow/node/node.h"
#include "signalflow/node/io/output/abstract.h"

#include <juce_audio_basics/juce_audio_basics.h>

namespace signalflow
{

class AudioOut_JUCE : public AudioOut_Abstract
{
public:
    AudioOut_JUCE();

    int init() override;
    int start() override;
    int stop() override;
    int destroy() override;

    void prepare_to_play(unsigned int sample_rate);
    void process_block(juce::AudioBuffer<float>& buffer);

private:
    std::string device_name;
};

REGISTER(AudioOut_JUCE, "audioout-juce")

} // namespace signalflow

#endif
