#pragma once
#include "Quat.h"

inline void quatToBodyGravity(const Quat& q, float& gx, float& gy, float& gz) {
    gx =  2.0f * (q.w * q.y - q.x * q.z);
    gy = -2.0f * (q.w * q.x + q.y * q.z);
    gz = -(q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
}
