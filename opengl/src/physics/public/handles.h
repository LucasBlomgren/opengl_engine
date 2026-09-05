#pragma once

#include "core/handle.h"

namespace physics {

struct BodyTag;
struct MotionStateTag;
struct SleepStateTag;
struct ColliderTag;

using BodyHandle = HandleT<BodyTag>;
using MotionStateHandle = HandleT<MotionStateTag>;
using SleepStateHandle = HandleT<SleepStateTag>;
using ColliderHandle = HandleT<ColliderTag>;

}
