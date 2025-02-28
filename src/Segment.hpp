#pragma once

#include <cmath>

struct Rider;

class Segment {
    private:
        double theta;

        double getFgravity(Rider& rider);
        double getFrolling(Rider& rider);
        double getFdrag(Rider& rider);

    public:
        const double grade;
        const double length;
        const double roadQuality = 0.005; //lower = better
        const double windSpeed = 0; //for now
        
        Segment(double grade, double length): grade(grade), length(length) {
            theta = atan(grade/100);
        };

        double getFresist(Rider& rider) {
            return getFgravity(rider) + getFrolling(rider) + getFdrag(rider);
        }
        const double& getGrade() const { return grade; }
};