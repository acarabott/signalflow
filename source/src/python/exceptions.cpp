#include "signalflow/python/python.h"

void init_python_exceptions(py::module &m)
{
    /*--------------------------------------------------------------------------------
     * Exceptions
     *-------------------------------------------------------------------------------*/
    py::register_exception<signalflow::graph_not_created_exception>(m, "GraphNotCreatedException");
    py::register_exception<signalflow::graph_already_created_exception>(m, "GraphAlreadyCreatedException");
    py::register_exception<signalflow::invalid_channel_count_exception>(m, "InvalidChannelCountException");
    py::register_exception<signalflow::patch_finished_playback_exception>(m, "PatchFinishedPlaybackException");
    py::register_exception<signalflow::device_not_found_exception>(m, "DeviceNotFoundException");
    py::register_exception<signalflow::audio_io_exception>(m, "AudioIOException");
}
