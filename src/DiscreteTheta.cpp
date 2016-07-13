#include "DiscreteTheta.hpp"

std::ostream& operator<< (std::ostream& stream, const DiscreteTheta& angle)
{
    stream << angle.getTheta();
    return stream;
}