#include "feature_proxy_base.h"

namespace data_access_engine {

bool FeatureProxyBase::is_editable() const {
    if (_parent == nullptr) {
        return true;
    }
    else {
        return _parent->is_editable();
    }
};

void FeatureProxyBase::mark_changed() {
    if (_parent != nullptr) {
        _parent->mark_changed();
    }
    ++_change_count;
};

bool FeatureProxyBase::cached_is_valid() const {
    if (_change_count == _valid_cache_count) {
        return _valid_cache;
    }
    _valid_cache = is_valid();
    _valid_cache_count = _change_count;
    return _valid_cache;
};

const FeatureWithIDProxyBase* FeatureProxyBase::container() const {
    if (_parent != nullptr) {
        return _parent->container();
    }
    return nullptr;
};

}; // data_access_engine