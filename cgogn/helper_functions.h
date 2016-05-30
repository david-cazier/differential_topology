#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H

namespace cgogn
{

template <typename T>
inline std::array<float,3> color_map_blue_green_red(T x)
{
	if (x < 0.0f)
		return {0.0f, 0.0f, 1.0f} ;

	if (x < 0.5f)
		return {0.0f, 2.0f * x, 1.0f - 2.0f * x};

	if (x < 1.0f)
		return {2.0f * x - 1.0f, 2.0f - 2.0f * x, 0.0f};

	return {1.0f, 0.0f, 0.0f};
}

template <typename T>
inline std::array<float,3> color_map_hash(T x)
{
	if (x < 0.0f)
		return {0.0f, 0.0f, 1.0f} ;

	if (x < 1.0f) {
		float hash = x*10-std::floor(x*10);
		return {x, hash, 1.0f-x};
	}
	return {1.0f, 1.0f, 1.0f};
}

}

#endif // HELPER_FUNCTIONS_H
