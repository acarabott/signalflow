/*------------------------------------------------------------------------
 * Buffer save example
 *
 * Demonstrates recording audio input (or any other synthesis node)
 * to a buffer, and saving the output to disk as a .wav file.
 *-----------------------------------------------------------------------*/

#include <iostream>
#include <signalflow/signalflow.h>
#include <unistd.h>

using namespace signalflow;

int main()
{
#if defined(HAVE_SOUNDIO) && defined(HAVE_SNDFILE)
    /*------------------------------------------------------------------------
     * Instantiate the global processing graph.
     *-----------------------------------------------------------------------*/
    AudioGraphRef graph = new AudioGraph();

    /*------------------------------------------------------------------------
     * Take a two-channel input from the default input device.
     *-----------------------------------------------------------------------*/
    NodeRef input = new AudioIn();

    /*------------------------------------------------------------------------
     * Create a five-second stereo buffer.
     *-----------------------------------------------------------------------*/
    BufferRef buffer = new Buffer(2, graph->get_sample_rate() * 5.0);

    /*------------------------------------------------------------------------
     * Create a BufferRecorder to perform a one-shot recording of the input.
     * It must be connected to the graph's output to trigger processing.
     *-----------------------------------------------------------------------*/
    NodeRef recorder = new BufferRecorder(buffer, input);
    graph->add_node(recorder);

    NodeRef rms = new RMS(input);
    graph->play(rms * 0.0);
    rms->poll(5);
    graph->start();

    /*------------------------------------------------------------------------
     * Wait until our recording is complete (happening in the audio I/O
     * thread), then save to disk.
     *-----------------------------------------------------------------------*/
    std::cout << "Starting recording..." << std::endl;
    usleep(5e6);
    std::cout << "Finished recording." << std::endl;

    buffer->save("out.wav");

#else
    std::cerr << "signalflow was not built with libsoundio and/or libsndfile" << std::endl;
    return 0;
#endif
}
