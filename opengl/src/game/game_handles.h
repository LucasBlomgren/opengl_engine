#pragma once

#include "core/handle.h"

struct GameObjectTag;
struct TransformTag;

using GameObjectHandle = HandleT<GameObjectTag>;
using TransformHandle = HandleT<TransformTag>;