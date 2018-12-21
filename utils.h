#pragma once

namespace utils {

	template<class T>
	inline constexpr T pow(const T base, const std::size_t exponent)
	{
		  return exponent == 0 ? 1 : base * pow(base, exponent - 1);
	}
	
}
