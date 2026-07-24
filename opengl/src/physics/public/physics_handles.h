#pragma once

#include "core/handle.h"

struct RigidBodyTag;
struct ColliderTag;

using RigidBodyHandle = HandleT<RigidBodyTag>;
using ColliderHandle = HandleT<ColliderTag>;