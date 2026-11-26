#pragma once

namespace ltr2 {

class Control {
public:
    virtual ~Control() = default;
    virtual void open() = 0;
    virtual void validate() = 0;
    virtual void measurement(bool enabled) = 0;
    virtual void streaming(bool enabled) = 0;
    virtual void close() noexcept = 0;
};

}
