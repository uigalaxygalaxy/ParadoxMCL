
#pragma once

#include <cstdint>
#include <vector>

struct Particle {
    float x;
    float y;
    float theta;
    float weight;
};

struct StartingPose {
    float x;
    float y;
    float theta;
};