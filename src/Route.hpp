#pragma once 

#include <Segment.hpp>
#include <Rider.hpp>
#include <Team.hpp>

class Route {
    private:
        std::vector<Segment> segments;
    public:
        void addSegment(Segment segment);
        void simulateRoute(std::vector<Team>& teams);
};