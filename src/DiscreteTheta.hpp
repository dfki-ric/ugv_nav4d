#pragma once
#include <assert.h>
#include <math.h>
#include <ostream>

class DiscreteTheta
{
    int theta;
    int numAngles;
    
    void normalize()
    {
        assert(numAngles >= 0);
        if(theta < 0)
        {
            int num = (theta / -numAngles) + 1;
            theta += numAngles * num;
        }

        if(theta >= numAngles)
            theta = theta % numAngles;
    }
    
public:
    DiscreteTheta(int val, unsigned int numAngles) : theta(val) , numAngles(numAngles) {
        normalize();
    }
    
    DiscreteTheta(double val, unsigned int numAngles) : numAngles(numAngles) {
        theta = floor(val / M_PI / 2.0 * numAngles);
        normalize();
    }
    
    DiscreteTheta(const DiscreteTheta &o) : theta(o.theta), numAngles(o.numAngles) {
    }
    
    DiscreteTheta& operator+=(const DiscreteTheta& rhs)
    {
        theta += rhs.theta;
        normalize();
        return *this;
    }

    DiscreteTheta& operator-=(const DiscreteTheta& rhs)
    {
        theta -= rhs.theta;
        normalize();
        return *this;
    }
    
    friend DiscreteTheta operator+(DiscreteTheta lhs, const DiscreteTheta& rhs)
    {
        lhs += rhs;
        return lhs;
    }

    friend DiscreteTheta operator-(DiscreteTheta lhs, const DiscreteTheta& rhs)
    {
        lhs -= rhs;
        return lhs;
    }

    friend bool operator<(const DiscreteTheta& l, const DiscreteTheta& r)
    {
        return l.theta < r.theta;
    }

    friend bool operator==(const DiscreteTheta& l, const DiscreteTheta& r)
    {
        return l.theta == r.theta;
    }
    
    int getTheta() const
    {
        return theta;
    }
    
    double getRadian() const
    {
        return M_PI * 2.0 * theta / static_cast<double>(numAngles);
    }
    
    
    DiscreteTheta shortestDist(const DiscreteTheta &ain) const
    {
        DiscreteTheta diffA = ain-*this;
        
        int a = diffA.theta;
        int b = numAngles - diffA.theta;
        
        
        if(a < b)
            return DiscreteTheta(a, numAngles);
        
        return DiscreteTheta(b, numAngles);
    }
};

std::ostream& operator<< (std::ostream& stream, const DiscreteTheta& angle);
