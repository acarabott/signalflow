#include "signalflow/node/io/output/abstract.h"

namespace signalflow
{

AudioOut_Abstract::AudioOut_Abstract()
{
    this->name = "audioout";
    // do we need to set num_output channels to allocate the right number of output buffers?
    this->set_channels(2, 0);
    this->no_input_upmix = true;
    this->has_variable_inputs = true;
    this->input_index = 0;
}

void AudioOut_Abstract::process(Buffer &out, int num_frames)
{
    for (int channel = 0; channel < this->num_input_channels; channel++)
    {
        memset(out[channel], 0, num_frames * sizeof(sample));
    }

    for (NodeRef input : this->audio_inputs)
    {
        for (int channel = 0; channel < input->get_num_output_channels(); channel++)
        {
#ifdef __APPLE__

            vDSP_vadd(input->out[channel], 1, out[channel], 1, out[channel], 1, num_frames);

#else

            for (int frame = 0; frame < num_frames; frame++)
            {
                out[channel][frame] += input->out[channel][frame];
            }

#endif
        }
    }
}

void AudioOut_Abstract::add_input(NodeRef node)
{
    audio_inputs.push_back(node);
    std::string input_name = "input" + std::to_string(input_index);
    this->input_index++;
    this->Node::create_input(input_name, audio_inputs.back());
}

void AudioOut_Abstract::remove_input(NodeRef node)
{
    bool removed = false;
    for (auto param : this->inputs)
    {
        if (*(param.second) == node)
        {
            this->destroy_input(param.first);
            audio_inputs.remove(node);
            removed = true;
            node->remove_output(this, param.first);
            break;
        }
    }
    if (!removed)
    {
        std::cerr << "Couldn't find node to remove" << std::endl;
    }
}

void AudioOut_Abstract::replace_input(NodeRef node, NodeRef other)
{
    bool replaced = false;
    for (auto param : this->inputs)
    {
        if (*(param.second) == node)
        {
            // Need to call create_input to also update the channel I/O on `other`
            audio_inputs.remove(node);
            audio_inputs.push_back(other);
            this->create_input(param.first, audio_inputs.back());
            replaced = true;
            break;
        }
    }

    if (!replaced)
    {
        std::cerr << "Couldn't find node to replace" << std::endl;
        // throw std::runtime_error("Couldn't find node to replace");
    }
}

std::list<NodeRef> AudioOut_Abstract::get_inputs()
{
    return this->audio_inputs;
}

unsigned int AudioOut_Abstract::get_sample_rate()
{
    return this->sample_rate;
}

unsigned int AudioOut_Abstract::get_buffer_size()
{
    return this->buffer_size;
}

} // namespace signalflow
