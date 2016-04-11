#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H

namespace cgogn
{

//inline float scale_to_0_1(float x, float min, float max)
//{
//    return (x - min) / (max - min);
//}


inline std::array<float,3> color_map_blue_green_red(float x)
{
    if (x < 0.0f)
        return {0.0f, 0.0f, 1.0f} ;

    if (x < 0.5f)
        return {0.0f, 2.0f * x, 1.0f - 2.0f * x};

    if (x < 1.0f)
        return {2.0f * x - 1.0f, 2.0f - 2.0f * x, 0.0f};

    return {1.0f, 0.0f, 0.0f};
}

}

#endif // HELPER_FUNCTIONS_H
