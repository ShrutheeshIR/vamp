#pragma once

#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <iostream>
#include <iomanip>

namespace vamp::planning
{
    template <std::size_t rake, std::size_t dim>
    inline static auto assignBlock(std::array<float, dim> src, vamp::FloatVector<rake, dim> &dest)
    {
        for (size_t i = 0; i < dim; i++)
        {
            dest[i] = src[i];
        }
    }
}  // namespace vamp::planning
