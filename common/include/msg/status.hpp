#pragma once

namespace msg
{

enum class status : int
{
    ok = 0,
    error = -1,
    invalid_arg = -2,
};

}  // namespace msg
