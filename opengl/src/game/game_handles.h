#pragma once

#include "physics/public/handles.h"

struct GameObjectTag;
struct TransformTag;

using GameObjectHandle = physics::HandleT<GameObjectTag>;
using TransformHandle = physics::HandleT<TransformTag>;