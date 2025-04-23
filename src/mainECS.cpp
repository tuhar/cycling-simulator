#include <string>
#include <vector>
#include <iostream>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <cmath>
#include <functional>

using Entity = u_int32_t;
using ComponentMask = u_int32_t;

struct NameComponent {
    std::string name;
    NameComponent() = default;
    NameComponent(std::string name): name(name){}
};

struct RiderComponent {
    u_int16_t weight;
    u_int16_t ftp;

    RiderComponent() = default;
    RiderComponent(u_int16_t weight, u_int16_t ftp):
    weight(weight),
    ftp(ftp) {}
};

struct SegmentComponent {
    float grade;
    float length;
    float roadQuality = 0.005;

    float theta;

    float windSpeed = 0;

    SegmentComponent() = default;
    SegmentComponent(float grade, float length): grade(grade), length(length), theta(atan(grade/100)) {};
};

struct EnergyComponent { 
    float greenEffort;
    float yellowEffort;
    float redEffort;

    float green;
    float yellow;
    float red;
    float black;

    EnergyComponent() = default;
    EnergyComponent(float ftp, float total)
        : greenEffort(ftp * 0.55),
          yellowEffort(ftp * 0.75),
          redEffort(ftp * 0.9),
          green(total * 0.55),
          yellow(total * 0.25),
          red(total * 0.15),
          black(total * 0.05) {}
};


struct SpeedComponent {
    float speed = 0.1;
};

struct DistanceComponent {
    Entity segment = -1;
    float distanceTraveledSegment = 0;
    float distanceTraveledTotal = 0;
    
};

struct EnergyConsumedComponent {
    float powerOutput = 0;
    float workDone = 0;
    float maxPower;

    EnergyConsumedComponent() = default;
    EnergyConsumedComponent(float ftp): maxPower(ftp * 1.1){}
};

float g = 9.8067;
float Cd = 0.63; //move to rider
float A = 0.509; //move to rider
float rho = 1.22601; // move to segment

enum ComponentType {
    RIDER = 1 << 0,
    SEGMENT = 1 << 1,
    ENERGY = 1 << 2,
    SPEED = 1 << 3,
    DISTANCE = 1 << 4,
    FATIGUE = 1 << 5,
    NAME = 1 << 6
};

struct EntityManager {
    Entity sequence = 0;

    std::vector<Entity> entities;
    std::unordered_map<Entity, NameComponent> names;

    std::unordered_map<Entity, RiderComponent> riders;
    std::unordered_set<Entity> ridersOnRoute;
    std::vector<Entity> finalClassification;

    std::unordered_map<Entity, SegmentComponent> segments;
    std::unordered_map<Entity, Entity> followingSegments;
    std::unordered_map<Entity, EnergyComponent> energies;

    std::unordered_map<Entity, SpeedComponent> speeds;
    std::unordered_map<Entity, DistanceComponent> distances;
    std::unordered_map<Entity, EnergyConsumedComponent> fatigues;
    
    std::unordered_map<Entity, ComponentMask> entityComponentMask;

    void createRider(std::string name, float weight, float ftp, float totalEnergy) {
        Entity entity = sequence++;
        entities.emplace_back(entity);
        riders.emplace(entity, RiderComponent(weight, ftp));
        ridersOnRoute.emplace(entity);

        energies.emplace(entity, EnergyComponent(ftp, totalEnergy));

        speeds.emplace(entity, SpeedComponent());

        distances.emplace(entity, DistanceComponent());

        fatigues.emplace(entity, EnergyConsumedComponent(ftp));

        names.emplace(entity, NameComponent(name));

        entityComponentMask[entity] |= RIDER | ENERGY | SPEED | DISTANCE | FATIGUE | NAME;
    }

    Entity createSegment(std::string name, float length, float grade) {
        Entity entity = sequence++;
        entities.emplace_back(entity);
        segments.emplace(entity, SegmentComponent(grade, length));
        names.emplace(entity, NameComponent(name));
        entityComponentMask[entity] |= SEGMENT | NAME;

        return entity;
    }
};


struct SpeedSystem {
    const ComponentMask requiredComponents = RIDER | DISTANCE | SPEED | FATIGUE;

    void update(EntityManager& em, float dt, float routeLength) {
        for (auto const& [entity, mask] : em.entityComponentMask) {
            if ((mask & requiredComponents) == requiredComponents) {
                auto& rider = em.riders[entity];
                auto& distance = em.distances[entity];
                auto& segment = em.segments[distance.segment];
                auto& speed = em.speeds[entity];
                auto& fatigue = em.fatigues[entity];

                float completition = (distance.distanceTraveledTotal/routeLength) * 100;
                float powerOutput = getPowerOutput(rider, fatigue, completition);
                float fResist = getFgravity(segment, rider) + getFrolling(segment, rider) + getFdrag(segment, speed);
                float acceleration = (powerOutput / (rider.weight * speed.speed)) - (fResist / rider.weight);
                speed.speed += dt * acceleration;

                //when more complex move to its own system
                float coveredDistance = speed.speed * dt;
                distance.distanceTraveledSegment += coveredDistance;
                distance.distanceTraveledTotal += coveredDistance;

                fatigue.workDone = fResist * coveredDistance;
                fatigue.powerOutput = powerOutput;
            }
        }
    }

    private:         
        float getPowerOutput(RiderComponent& rider, EnergyConsumedComponent& energyConsumedComponent,  float completition) {
            if (completition < 10) return energyConsumedComponent.maxPower;  // Start strong
            if (completition < 90) return rider.ftp * 0.50;  // Steady pace
            return energyConsumedComponent.maxPower;                      // Sprint finish
        }

        float getFgravity(SegmentComponent& segment, RiderComponent& rider) {
            return g * (sin(segment.theta)) * rider.weight;
        };
        
        float getFrolling(SegmentComponent& segment, RiderComponent& rider) {
            return g * cos(segment.theta) * rider.weight * segment.roadQuality;
        }
        float getFdrag(SegmentComponent& segment, SpeedComponent& speedComponent) {
            return 0.5 * Cd * A * rho * pow((speedComponent.speed + segment.windSpeed), 2.0);
        } 
};

struct SegmentSystem {
    const ComponentMask requiredComponents = DISTANCE;

    void init(EntityManager& em, Entity& firstSegment) {
        for (auto const& [entity, mask] : em.entityComponentMask) {
            if ((mask & requiredComponents) == requiredComponents) {
                auto& distance = em.distances[entity];
                distance.segment = firstSegment;
            }
        }
    }

    void update(EntityManager& em) {
        for (auto const& [entity, mask] : em.entityComponentMask) {
            if ((mask & requiredComponents) == requiredComponents) {
                auto& distance = em.distances[entity];
                auto& segment = em.segments[distance.segment];
                if (distance.distanceTraveledSegment >= segment.length) {
                    auto nextSegment = em.followingSegments.find(distance.segment);
                    if (nextSegment != em.followingSegments.end()) {
                        float overflow = distance.distanceTraveledSegment - segment.length;
                        distance.distanceTraveledSegment = overflow;
                        distance.segment = nextSegment->second;
                    } else {
                        em.ridersOnRoute.erase(entity);
                        em.entityComponentMask[entity] &= ~DISTANCE; 
                        em.finalClassification.emplace_back(entity);
                    }
                }
            }
        }
    }
};

struct FatigueSystem {
    const ComponentMask requiredComponents = RIDER | ENERGY | FATIGUE;

    void update(EntityManager& em) {
        for (auto const& [entity, mask] : em.entityComponentMask) {
            if ((mask & requiredComponents) == requiredComponents) {
                auto& energy = em.energies[entity];
                auto& fatigue = em.fatigues[entity];
                auto& rider = em.riders[entity];
                if(updateFatigue(fatigue, energy)) {
                    updateMaxPower(fatigue, energy, rider);
                };
            }
        }
    }

    private: 
        void updateMaxPower(EnergyConsumedComponent& fatigueComponent, EnergyComponent& energyComponent, RiderComponent& riderComponent) {
            if (energyComponent.green <= 0) {
                fatigueComponent.maxPower = riderComponent.ftp * 0.15;
            } else if (energyComponent.yellow <= 0) {
                fatigueComponent.maxPower = riderComponent.ftp * 0.55;
            } else if (energyComponent.red <= 0) {
                fatigueComponent.maxPower = riderComponent.ftp * 0.95;
            } else if (energyComponent.black <= 0) {
                fatigueComponent.maxPower = riderComponent.ftp;
            }
        }

        bool updateFatigue(EnergyConsumedComponent& fatigueComponent, EnergyComponent& energyComponent) {
            if (fatigueComponent.powerOutput <= energyComponent.greenEffort) {
                return updateGreen(energyComponent, fatigueComponent);
            } else if (fatigueComponent.powerOutput > energyComponent.greenEffort && fatigueComponent.powerOutput <= energyComponent.yellowEffort) {
                return updateYellow(energyComponent, fatigueComponent);
            } else if (fatigueComponent.powerOutput > energyComponent.yellowEffort && fatigueComponent.powerOutput <= energyComponent.redEffort) {
                return updateRed(energyComponent, fatigueComponent);
            } else {
                return updateBlack(energyComponent, fatigueComponent);
            }
        }

        bool updateGreen(EnergyComponent& energyComponent, EnergyConsumedComponent& fatigueComponent) {
            if (energyComponent.green <= 0) {
                return false;
            }
            energyComponent.green -= fatigueComponent.workDone;
            return energyComponent.green <= 0;
        };
        bool updateYellow(EnergyComponent& energyComponent, EnergyConsumedComponent& fatigueComponent) {
            if (energyComponent.yellow <= 0) {
                return false;
            }
            energyComponent.yellow -= fatigueComponent.workDone;
            return energyComponent.yellow <= 0;
        };
        bool updateRed(EnergyComponent& energyComponent, EnergyConsumedComponent& fatigueComponent) {
            if (energyComponent.red <= 0) {
                return false;
            }
            energyComponent.red-= fatigueComponent.workDone;
            return energyComponent.red <= 0;
        };
        bool updateBlack(EnergyComponent& energyComponent, EnergyConsumedComponent& fatigueComponent) {
            if (energyComponent.black <= 0) {
                return false;
            }
            energyComponent.black -= fatigueComponent.workDone;
            return energyComponent.black <= 0;
        }
};

int main(int argc, char const *argv[])
{
    EntityManager em;
    
    SpeedSystem speedSystem;
    FatigueSystem fatigueSystem;
    SegmentSystem segmentSystem;

    em.createRider("Joonas", 69, 400, 2500000);
    em.createRider("Pogi", 66, 450, 2250000);
    em.createRider("MvP", 75, 400, 2000000);
    em.createRider("Remco", 73, 400, 1950000);

    // extract to level json? 
    Entity firstSegment = em.createSegment("Dedinka vo Flandroch", 10000, 0);
    segmentSystem.init(em, firstSegment);
    Entity krpal = em.createSegment("Krpal", 5000, 6);
    em.followingSegments.emplace(firstSegment, krpal);
    Entity serpentinky = em.createSegment("Serpentinky", 5000, -3);
    em.followingSegments.emplace(krpal, serpentinky);
    Entity rovinecka = em.createSegment("Rovinecka", 15000, 0);
    em.followingSegments.emplace(serpentinky, rovinecka);
    Entity kopcovityFinish = em.createSegment("Alp'duez", 17000, 12);
    em.followingSegments.emplace(rovinecka, kopcovityFinish);

    float dt = 0.1;
    float time = 0;

    while (!em.ridersOnRoute.empty()) {
        speedSystem.update(em, dt, 52000);
        fatigueSystem.update(em);
        segmentSystem.update(em);

        time += dt;
    }

    //extract to system?
    int i = 1;
    for (const auto& entity : em.finalClassification) {
        std::cout << i++ << ". place: " << em.names[entity].name << " energy left: " << em.energies[entity].green << "J" << std::endl;
    }

    return 0;
}
