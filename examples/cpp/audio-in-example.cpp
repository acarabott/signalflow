/*------------------------------------------------------------------------
 * Audio input example
 *
 * Takes audio input from the default input device, records it to
 * a short buffer, and plays it back at variable rates.
 * 
 * Make some noise when starting the patch.
 *-----------------------------------------------------------------------*/

#include <signalflow/signalflow.h>

using namespace signalflow;

int main()
{
#ifdef HAVE_SOUNDIO
    /*------------------------------------------------------------------------
     * Instantiate a global processing graph.
     *-----------------------------------------------------------------------*/
    AudioGraphRef graph = new AudioGraph();

    /*------------------------------------------------------------------------
     * Take a two-channel input from the default input device.
     *-----------------------------------------------------------------------*/
    NodeRef input = new AudioIn();

    /*------------------------------------------------------------------------
     * Create a one-second stereo buffer.
     *-----------------------------------------------------------------------*/
    BufferRef buffer = new Buffer(2, graph->get_sample_rate() * 1.0);

    /*------------------------------------------------------------------------
     * Create a BufferRecorder to perform a one-shot recording of the input.
     *-----------------------------------------------------------------------*/
    NodeRef recorder = new BufferRecorder(buffer, input);
    graph->play(recorder);

    /*------------------------------------------------------------------------
     * Create a parallel series of BufferPlayers to play back the recording.
     * Add a varispeed effect with interpolated WhiteNoise objects assigned to
     * the sampler's rate input.
     *-----------------------------------------------------------------------*/
    for (int i = 0; i < 5; i++)
    {
        NodeRef rate = new WhiteNoise(0.5, 0.3, 1.8, true);
        NodeRef sampler = new BufferPlayer(buffer, rate, true);

        /*------------------------------------------------------------------------
         * Attenuate the output level so we don't distort.
         *-----------------------------------------------------------------------*/
        graph->play(sampler * 0.2);
    }

    graph->wait();
#else
    std::cerr << "signalflow was not built with libsoundio" << std::endl;
    return 0;
#endif
}
