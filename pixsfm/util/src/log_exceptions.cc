#include <sstream>

#include <colmap/sensor/models.h>
#include "log_exceptions.h"

std::ostream &operator<<(std::ostream &os, const colmap::CameraModelId &id) {
    os << static_cast<int>(id);
    return os;
}
