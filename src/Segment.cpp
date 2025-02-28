#include <Segment.hpp>
#include <Rider.hpp>

const double g = 9.8067;
const double Cd = 0.63; //move to rider
const double A = 0.509; //move to rider
const double rho = 1.22601; // move to segment

double Segment::getFgravity(Rider& rider) {
    return g * (sin(theta)) * rider.weight;
};

double Segment::getFrolling(Rider& rider) {
    return g * cos(theta) * rider.weight * roadQuality;
}
double Segment::getFdrag(Rider& rider) {
    return 0.5 * Cd * A * rho * pow((rider.speed + windSpeed), 2.0);
}  