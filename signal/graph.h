#pragma once 

#include "node.h"

namespace libsignal
{
	class AudioOut_Abstract;

	class Graph
	{
		public:

			Graph();

			void run();

			/**------------------------------------------------------------------------
			 * Perform batch (offline) processing of a given node graph.
			 *
			 *------------------------------------------------------------------------*/
			void process(const NodeRef &root, int num_frames, int block_size = SIGNAL_DEFAULT_BLOCK_SIZE);

			void pull_input(const NodeRef &node, int num_frames);
			void pull_input(int num_frames);

			NodeRef add_node(Node *node);

			NodeRef input = nullptr;
			NodeRef output = nullptr;

			float sample_rate;

		private: 

			std::set<Node *> processed_nodes;
	};

    class GraphRef : public std::shared_ptr<Graph>
    {
        public:
            using std::shared_ptr<Graph>::shared_ptr;

            GraphRef() : std::shared_ptr<Graph>(nullptr) { }
            GraphRef(Graph *ptr) : std::shared_ptr<Graph>(ptr) { }
    };
}
