#include "config_address.h"

namespace data_access_engine {

std::unique_ptr<ConfigAddress> ConfigAddress::_s_config_instance;

ConfigAddress* ConfigAddress::get_instance()
{
    if (!_s_config_instance) {
        _s_config_instance.reset(new ConfigAddress);
    }
    return _s_config_instance.get();
};

}; //data_access_engine
