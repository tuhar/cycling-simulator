#pragma once

#include <vector>
#include <string>
#include <iostream>
#include <Energy.hpp>

class Segment;
struct Rider {
    u_int32_t id;

    Energy energy; // in J
    double weight; //for now with all the gear
    std::string name;
    u_int32_t ftp;

    double greenEffort;
    double yellowEffort;
    double redEffort;

    double speed = 0.1;
    std::vector<Segment, std::allocator<Segment>>::iterator segment;
    double distanceTotal = 0;
    double distanceSegment = 0;

    Rider(u_int32_t id, std::string name, double weight,u_int32_t ftp, double energy = 8000000): id(id), name(name), weight(weight), ftp(ftp), energy(Energy(energy)) {
        greenEffort = ftp * 0.55;
        yellowEffort = ftp * 0.75;
        redEffort = ftp * 0.9;
    };

    double getPowerOutput(double distance) const;
    bool updateFatigue(const double workDone, const double powerOutput);        
    void nextSegment(double distanceOverflow);    
    bool updateRider(double time, double dt);
};