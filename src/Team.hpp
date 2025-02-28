#pragma once 
#include <Rider.hpp>

struct Team {
    std::vector<Rider> riders;
    std::string name;
    
    Team(std::string name): name(name){};        
    void addRider(Rider rider);
};