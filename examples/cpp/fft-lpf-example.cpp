/*------------------------------------------------------------------------
 * FFT brick-wall filter example.
 *
 * Performs an FFT on an incoming signal, then passes it through
 * a frequency-domain brick wall filter, zeroing any bins beyond
 * the specified cutoff frequency.
 *-----------------------------------------------------------------------*/
#include <signalflow/signalflow.h>

using namespace signalflow;

int main()
{
    /*------------------------------------------------------------------------
     * Create the global signal processing graph.
     *-----------------------------------------------------------------------*/
    AudioGraphRef graph = new AudioGraph();

    /*------------------------------------------------------------------------
     * Load and play a sample.
     *-----------------------------------------------------------------------*/
    BufferRef buffer = new Buffer("audio/gliss.aif");
    NodeRef sampler = new BufferPlayer(buffer, 1.0, true);

    /*------------------------------------------------------------------------
     * Perform FFT -> filter -> inverse FFT
     *-----------------------------------------------------------------------*/
    NodeRef fft = new FFT(sampler);
    NodeRef lpf = new FFTLPF(fft, 400);
    NodeRef output = new IFFT(lpf);

    /*------------------------------------------------------------------------
     * Pan the output to centre of stereo field.
     *-----------------------------------------------------------------------*/
    NodeRef pan = new StereoPanner(output);

    /*------------------------------------------------------------------------
     * Send to output.
     *-----------------------------------------------------------------------*/
    graph->add_output(pan);
    graph->start();
    graph->wait();
}
