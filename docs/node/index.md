# Nodes

A `Node` object is an audio processing unit that performs one single function. For example, a Node's role may be to synthesize a waveform, read from a buffer, or take two input Nodes and sum their values.

- Nodes are [played and stopped](/node/playback) by connecting them to the AudioGraph 
- A node has one or more [audio-rate inputs](/node/inputs#audio-rate-inputs), which can be modulated by other nodes
- Some nodes can be triggered with [trigger inputs](/node/inputs#triggers) — for example, to restart playback, or set the position of an envelope
- Some nodes can be used to play back the contents of [buffer inputs](/node/inputs#buffer-inputs), or can use buffer data as a source of modulation
- The output of multiple nodes can be combined and modulated with use of the standard Python [operators](/node/operators) (`+`, `-`, `*`, `%`, etc)
- The output of a node can be mono (single-channel) or [multichannel](/node/multichannel)
- A Node's status and output can be examined by querying its [properties](/node/properties)
- Some Nodes generate unpredictable [stochastic output](/node/stochastic), which can be controlled via its internal random number generator 
- Details of how to create a new Node type are detailed in [Developing a new Node class](/node/developing)
- For an overview of every type of Node available in SignalFlow, see the [Node Reference Library](/node/library)

---

[→ Next: Node playback](/node/playback)