#include "signalflow/node/io/output/juce.h"

#include <iostream>

namespace signalflow
{

extern AudioGraph *shared_graph;

AudioOut_JUCE::AudioOut_JUCE()
    : AudioOut_Abstract()
{
    this->name = "audioout-juce";
    this->sample_rate = 44100;
}

int AudioOut_JUCE::init()
{
    return 0;
}

int AudioOut_JUCE::start()
{
    this->set_state(SIGNALFLOW_NODE_STATE_ACTIVE);
    return 0;
}

int AudioOut_JUCE::stop()
{
    this->set_state(SIGNALFLOW_NODE_STATE_STOPPED);
    return 0;
}

int AudioOut_JUCE::destroy()
{
    return 0;
}

void AudioOut_JUCE::prepare_to_play(const unsigned int sample_rate)
{
    this->sample_rate = sample_rate;
    shared_graph->set_sample_rate(static_cast<int>(sample_rate));
}

void AudioOut_JUCE::process_block(juce::AudioBuffer<float> &buffer)
{
    const auto num_frames = buffer.getNumSamples();

    try
    {
        shared_graph->render(num_frames);
    }
    catch (const std::exception &exception)
    {
        std::cerr << "Exception in AudioGraph: " << exception.what() << std::endl;
        exit(1);
    }

    const auto output = shared_graph->get_output();

    for (auto channel = 0; channel < buffer.getNumChannels(); channel++)
    {
        memcpy(buffer.getWritePointer(channel),
               shared_graph->get_output()->out[channel],
               static_cast<size_t>(num_frames) * static_cast<size_t>(sizeof(sample)));
    }
}

} // namespace signalflow
