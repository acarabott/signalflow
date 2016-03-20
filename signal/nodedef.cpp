#include "nodedef.h"

namespace libsignal
{
	NodeDefinition::NodeDefinition() : name("(null)")
	{
	}

	NodeDefinition::~NodeDefinition()
	{
		// free params
	}

	NodeDefinition::NodeDefinition(std::string name) : name(name)
	{
	}

	NodeDefinition::NodeDefinition(std::string name, float value) : name(name)
	{
		this->set_value(value);
	}

	void NodeDefinition::add_param(std::string name, NodeDefinition *def)
	{
		NodeDefinition *def_copy = new NodeDefinition();
		*def_copy = *def;
		this->params[name] = def_copy;
	}

	void NodeDefinition::add_param(std::string name, float value)
	{
		this->params[name] = new NodeDefinition("constant", value);
	}

	void NodeDefinition::set_value (float value)
	{
		this->value = value;
		this->is_constant = true;
	}

};
