#pragma once 

#include "unit.h"

namespace signum
{

	class Graph
	{
		public:

			Graph();

			void run();
			void pull_input(UnitRef &unit, int num_frames);
			void pull_input(int num_frames);
			UnitRef addUnit(Unit *unit);

			UnitRef input = nullptr;
			UnitRef output = nullptr;
	};
}
