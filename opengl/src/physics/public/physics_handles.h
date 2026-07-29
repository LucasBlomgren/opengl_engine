#pragma once

#include "core/handle.h"

namespace physics {

struct BodyTag;
struct ColliderTag;

using BodyHandle = HandleT<BodyTag>;
using ColliderHandle = HandleT<ColliderTag>;

}
