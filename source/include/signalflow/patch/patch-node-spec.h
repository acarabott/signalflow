#pragma once

#include <string>
#include <unordered_map>
#include <signalflow/core/property.h>

namespace signalflow
{

class PatchNodeSpec
{
public:
    ~PatchNodeSpec();
    PatchNodeSpec(std::string name = "null");
    PatchNodeSpec(std::string name, float value);

    int get_id();
    void set_id(int value);

    std::string get_name();
    void set_name(std::string name);

    bool get_is_constant();
    float get_constant_value();
    void set_constant_value(float value);

    /**--------------------------------------------------------------------------------
     *
     *--------------------------------------------------------------------------------*/
    std::string get_input_name();
    void set_input_name(std::string name);

    std::unordered_map<std::string, PatchNodeSpec *> get_inputs();
    std::unordered_map<std::string, std::string> get_buffer_inputs();

    void add_input(std::string name, PatchNodeSpec *def);
    void add_input(std::string name, float value);
    void add_buffer_input(std::string patch_input_name,
                          std::string node_input_name);

    std::unordered_map<std::string, PropertyRef> get_properties();
    void add_property(std::string name, PropertyRef property);

private:
    std::string name = "";
    int id = -1;
    float value = 0.0;
    bool is_constant = false;
    std::string input_name;
    std::unordered_map<std::string, PatchNodeSpec *> inputs;
    std::unordered_map<std::string, std::string> buffer_inputs;
    std::unordered_map<std::string, PropertyRef> properties;
};

}
