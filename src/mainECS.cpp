#include <string>
#include <vector>
#include <iostream>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <cmath>
#include <functional>
#include <typeindex>
#include <memory>
#include <any>

#include <random>
#include <chrono>

using Entity = u_int32_t;
using ComponentMask = u_int32_t;
struct LevelDetails {
    float routeLenght = 0;
    float raceTime = 0;
    std::vector<Entity> classification;
    std::unordered_set<Entity> ridersOnRoute;
};

struct ComponentMaskFrame {
    ComponentMask currentFrame;
    ComponentMask nextFrame;

    ComponentMaskFrame(ComponentMask currentFrame, ComponentMask nextFrame): currentFrame(currentFrame), nextFrame(nextFrame){}
};

struct NameComponent {
    std::string name;
    NameComponent(std::string name): name(name){}
};

struct RiderComponent {
    u_int16_t weight;
    u_int16_t ftp;

    RiderComponent(u_int16_t weight, u_int16_t ftp):
    weight(weight),
    ftp(ftp) {}
};

struct LengthComponent {
    float length;

    LengthComponent(float length): length(length) {};
};

struct NextSegmentComponent {
    Entity nextSegment;

    NextSegmentComponent(Entity nextSegment): nextSegment(nextSegment) {};
};

struct SegmentComponent {
    float roadQuality = 0.005;
    float theta;
    float windSpeed = 0;

    SegmentComponent(float grade): theta(atan(grade/100)) {};
};

struct EnergyComponent { 
    float greenEffort;
    float yellowEffort;
    float redEffort;

    float green;
    float yellow;
    float red;
    float black;

    bool greenDone = false;
    bool yellowDone = false;
    bool redDone = false;
    bool blackDone = false;

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
    Entity segment;
    double distanceRemainingOnSegment;
    double distanceRemainingTotal;
    
    DistanceComponent(Entity segment, double segmentLength, double totalLength): segment(segment), distanceRemainingOnSegment(segmentLength), distanceRemainingTotal(totalLength) {};
};

struct FatigueComponent {
    float powerOutput = 0;
    float workDone = 0;
    float maxPower;

    FatigueComponent(float ftp): maxPower(ftp * 1.1){}
};

float g = 9.8067;
float Cd = 0.63; //drag coefficient -> todo move to rider
float A = 0.509; //frontal area -> todo move to rider
float rho = 1.22601; //density of air -> todo move to segment

enum ComponentType {
    RIDER = 1 << 0,
    ENERGY = 1 << 1,
    SPEED = 1 << 2,
    DISTANCE = 1 << 3,
    FATIGUE = 1 << 4,
    NAME = 1 << 5,
    LENGTH = 1 << 6,
    NEXT_SEGMENT = 1 << 7,
    SEGMENT = 1 << 8,
};

template<typename Component>
using ComponentViewVector = std::vector<Component*>;
template<typename... Components>
using ComponentViewVectorTuple = std::tuple<ComponentViewVector<Components>...>;

template<typename... Components>
struct UpdateView {
    std::vector<Entity> entities;
    std::unordered_map<Entity, size_t> entityToIndex;

    ComponentViewVectorTuple<Components...> components;

    void add(Entity entity, Components*... entityComponents) {
        entities.push_back(entity);
        entityToIndex[entity] = entities.size() - 1;
        addComponents(std::index_sequence_for<Components...>{}, entityComponents...);
    }

    template<typename TargetComponent>
    ComponentViewVector<TargetComponent>& getComponent() {
        return std::get<ComponentViewVector<TargetComponent>>(components);
    }

    void remove(Entity entity) {
        auto it = entityToIndex.find(entity);
        if (it == entityToIndex.end()) return;

        size_t index = it->second;
        size_t lastIndex = entities.size() - 1;

        removeComponents(index, lastIndex, std::index_sequence_for<Components...>{});
        std::swap(entities[index], entities[lastIndex]);
        entities.pop_back();
        entityToIndex[entities[index]] = index;
        entityToIndex.erase(entity);
    }

    private:
        template<std::size_t... Is>
        void addComponents(std::index_sequence<Is...>, Components*... entityComponents) {
            (..., (std::get<Is>(components).push_back(entityComponents)));
        }

        template<std::size_t... Is>
        void removeComponents(size_t index, size_t lastIndex, std::index_sequence<Is...>) {
            (..., (
                std::swap(std::get<Is>(components)[index], std::get<Is>(components)[lastIndex]),
                std::get<Is>(components).pop_back()
            ));
        }
};

struct ComponentManager {
   std::unordered_map<std::type_index, std::any> entityComponents;

    template <typename Component>
    void registerComponent() {
        entityComponents[typeid(Component)] = std::unordered_map<Entity, Component>();
    }

    template<typename Component>
    void addComponent(Entity entity, Component&& component) {
        std::type_index type = typeid(std::decay_t<Component>);
        auto it = entityComponents.find(type);
        if(it == entityComponents.end()) {
            throw std::runtime_error("Component " + std::string(type.name()) + " not registered for entity: " + std::to_string(entity) +" !");
        };

        auto& componentMap = std::any_cast<std::unordered_map<Entity, std::decay_t<Component>>&>(it->second);
        componentMap.emplace(entity, std::forward<Component>(component));
    }

    template<typename Component>
    Component& getComponent(Entity entity) {
        std::type_index type = typeid(Component);
        auto it = entityComponents.find(type);
        if (it == entityComponents.end()) {
            throw std::runtime_error("Component " + std::string(type.name()) + " not registered for entity: " + std::to_string(entity) +" !");
        }

        auto& componentMap = std::any_cast<std::unordered_map<Entity, Component>&>(it->second);
        auto compIt = componentMap.find(entity);
        if (compIt == componentMap.end()) {
            throw std::runtime_error("Component " + std::string(type.name()) + " not found for entity: " + std::to_string(entity) +" !");
        }
        return compIt->second;
    }

};

struct EntityManager {
    Entity sequence = 0;    
    std::unordered_map<Entity, ComponentMaskFrame> entityComponentMask;

    bool hasComponent(Entity entity, ComponentMask requiredComponent) {
        return entityComponentMask.at(entity).currentFrame && requiredComponent == requiredComponent;
    }

    Entity createEntity(ComponentMask signature) {
        Entity entity = sequence++;
        entityComponentMask.emplace(entity, ComponentMaskFrame(0, signature));

        return entity;
    }
};

struct System {
    EntityManager& em;
    ComponentManager& cm;

    ComponentMask componentMask;

    System(EntityManager& em, ComponentManager& cm, ComponentMask componentMask): em(em), cm(cm), componentMask(componentMask){};
    virtual ~System() = default;

    virtual void update(float dt, LevelDetails& level) = 0;
    virtual void addEntityToView(Entity entity) = 0;
    virtual void removeEntityFromView(Entity entity) = 0;
};
struct SystemManager {
    std::vector<std::unique_ptr<System>> systems;

    template<typename T, typename... Args>
    void registerSystem(Args&&... args) {
        systems.emplace_back(std::make_unique<T>(std::forward<Args>(args)...));
    }

    void update(float dt, LevelDetails& level) {
        for (auto& system : systems) {
            // auto speedTickStart = std::chrono::high_resolution_clock::now();
            system->update(dt, level);
            // auto speedTickStop = std::chrono::high_resolution_clock::now();
            // std::chrono::duration<double, std::milli> speedDruation = speedTickStop - speedTickStart;
            // std::cout << "system updated in " << speedDruation.count() << "ms" << std::endl;
        }
    }

    void updateComponentMask(EntityManager& em) {
        for (auto& [entity, componentMaskFrame] : em.entityComponentMask) {
            for (auto& system: systems) {
                if ((componentMaskFrame.currentFrame & system->componentMask) != system->componentMask &&
                    (componentMaskFrame.nextFrame & system->componentMask) == system->componentMask) {
                    system->addEntityToView(entity);
                }
                if ((componentMaskFrame.currentFrame & system->componentMask) == system->componentMask &&
                    (componentMaskFrame.nextFrame & system->componentMask) != system->componentMask) {
                    system->removeEntityFromView(entity);
                }
            }
            componentMaskFrame.currentFrame = componentMaskFrame.nextFrame;
        }
    }
};

struct SpeedSystem: System {
    UpdateView<RiderComponent, DistanceComponent, SpeedComponent, FatigueComponent> updateView;    
    SpeedSystem(EntityManager& em, ComponentManager& cm): System(em, cm, RIDER | DISTANCE | SPEED | FATIGUE){};

    void update(float dt, LevelDetails& level) {
        for (size_t i = 0; i < updateView.entities.size(); i++) {
                RiderComponent* rider = updateView.getComponent<RiderComponent>().at(i);
                DistanceComponent* distance = updateView.getComponent<DistanceComponent>().at(i);
                SegmentComponent& segment = cm.getComponent<SegmentComponent>(distance->segment);
                SpeedComponent* speed = updateView.getComponent<SpeedComponent>().at(i);
                FatigueComponent* fatigue = updateView.getComponent<FatigueComponent>().at(i);

                float completition = (distance->distanceRemainingTotal/level.routeLenght) * 100; //todo fix inverted distance
                float powerOutput = getPowerOutput(rider, fatigue, completition);
                float fResist = getFgravity(segment, rider) + getFrolling(segment, rider) + getFdrag(segment, speed);
                float acceleration = (powerOutput / (rider->weight * speed->speed)) - (fResist / rider->weight);
                speed->speed += dt * acceleration;

                //when more complex move to its own system
                double coveredDistance = speed->speed * dt;
                distance->distanceRemainingOnSegment -= coveredDistance;
                distance->distanceRemainingTotal -= coveredDistance;

                fatigue->workDone = fResist * coveredDistance;
                fatigue->powerOutput = powerOutput;
        }
    }
    void addEntityToView(Entity entity){
        auto& rider = cm.getComponent<RiderComponent>(entity);
        auto& distance = cm.getComponent<DistanceComponent>(entity);
        auto& speed = cm.getComponent<SpeedComponent>(entity);
        auto& fatigue = cm.getComponent<FatigueComponent>(entity);
        updateView.add(entity, &rider, &distance, &speed, &fatigue);
    };
    void removeEntityFromView(Entity entity) {
        updateView.remove(entity);
    };

    private:         
        float getPowerOutput(RiderComponent* rider, FatigueComponent* energyConsumedComponent, float completition) {
            if (completition > 90) return energyConsumedComponent->maxPower;  // Start strong
            if (completition > 10) return rider->ftp * 0.50;  // Steady pace
            return energyConsumedComponent->maxPower;                      // Sprint finish
        }

        float getFgravity(SegmentComponent& segment, RiderComponent* rider) {
            return g * (sin(segment.theta)) * rider->weight;
        };
        
        float getFrolling(SegmentComponent& segment, RiderComponent* rider) {
            return g * cos(segment.theta) * rider->weight * segment.roadQuality;
        }
        float getFdrag(SegmentComponent& segment, SpeedComponent* speedComponent) {
            return 0.5 * Cd * A * rho * pow((speedComponent->speed + segment.windSpeed), 2.0);
        } 
};

struct SegmentSystem: System {
    UpdateView<DistanceComponent> updateView;    
    SegmentSystem(EntityManager& em, ComponentManager& cm): System(em, cm, DISTANCE){};

    void update(float dt, LevelDetails& level) {        
        for (size_t i = 0; i < updateView.entities.size(); i++) {
                Entity& entity = updateView.entities.at(i);
                DistanceComponent* distance = updateView.getComponent<DistanceComponent>().at(i);
                if (distance->distanceRemainingTotal <= 0) {
                    em.entityComponentMask.at(entity).nextFrame &= ~(DISTANCE | FATIGUE | SPEED);
                    level.classification.emplace_back(entity);
                    level.ridersOnRoute.erase(entity);
                } else if (distance->distanceRemainingOnSegment <= 0) {
                    NextSegmentComponent& nextSegmentComponent = cm.getComponent<NextSegmentComponent>(distance->segment);
                    LengthComponent& nextSegmentLength = cm.getComponent<LengthComponent>(nextSegmentComponent.nextSegment);
                    distance->distanceRemainingOnSegment += nextSegmentLength.length;
                    distance->segment = nextSegmentComponent.nextSegment;
                }
        }
    }

    void addEntityToView(Entity entity){
        auto& distance = cm.getComponent<DistanceComponent>(entity);
        updateView.add(entity, &distance);
    };
    void removeEntityFromView(Entity entity) {
        updateView.remove(entity);
    };
};

struct FatigueSystem: System {
    UpdateView<EnergyComponent, FatigueComponent, RiderComponent> updateView;
    FatigueSystem(EntityManager& em, ComponentManager& cm): System(em, cm, ENERGY | FATIGUE | RIDER){};

    void update(float dt, LevelDetails& level) {
        for (size_t i = 0; i < updateView.entities.size(); i++) {
            EnergyComponent* energy = updateView.getComponent<EnergyComponent>().at(i);
            FatigueComponent* fatigue = updateView.getComponent<FatigueComponent>().at(i);
            RiderComponent* rider = updateView.getComponent<RiderComponent>().at(i);
            if (updateFatigue(fatigue, energy)) {
                updateMaxPower(fatigue, energy, rider);
            };
        }
    }

    void addEntityToView(Entity entity){
        auto& rider = cm.getComponent<RiderComponent>(entity);
        auto& energy = cm.getComponent<EnergyComponent>(entity);
        auto& fatigue = cm.getComponent<FatigueComponent>(entity);
        updateView.add(entity, &energy, &fatigue, &rider);
    };
    void removeEntityFromView(Entity entity) {
        updateView.remove(entity);
    };

    private: 
        void updateMaxPower(FatigueComponent* fatigueComponent, EnergyComponent* energyComponent, RiderComponent* riderComponent) {
            if (energyComponent->green <= 0 && !energyComponent->greenDone) {
                fatigueComponent->maxPower = riderComponent->ftp * 0.15;
                energyComponent->greenDone = true;
            } else if (energyComponent->yellow <= 0 && !energyComponent->yellowDone) {
                fatigueComponent->maxPower = riderComponent->ftp * 0.55;
                energyComponent->yellowDone = true;
            } else if (energyComponent->red <= 0 && !energyComponent->redDone) {
                fatigueComponent->maxPower = riderComponent->ftp * 0.95;
                energyComponent->redDone = true;
            } else if (energyComponent->black <= 0 && !energyComponent->blackDone) {
                fatigueComponent->maxPower = riderComponent->ftp;
                energyComponent->blackDone = true;
            }
        }

        bool updateFatigue(FatigueComponent* fatigueComponent, EnergyComponent* energyComponent) {
            if (fatigueComponent->powerOutput <= energyComponent->greenEffort) {
                return updateGreen(energyComponent, fatigueComponent);
            } else if (fatigueComponent->powerOutput > energyComponent->greenEffort && fatigueComponent->powerOutput <= energyComponent->yellowEffort) {
                return updateYellow(energyComponent, fatigueComponent);
            } else if (fatigueComponent->powerOutput > energyComponent->yellowEffort && fatigueComponent->powerOutput <= energyComponent->redEffort) {
                return updateRed(energyComponent, fatigueComponent);
            } else {
                return updateBlack(energyComponent, fatigueComponent);
            }
        }

        bool updateGreen(EnergyComponent* energyComponent, FatigueComponent* fatigueComponent) {
            if (energyComponent->green <= 0) {
                return false;
            }
            energyComponent->green -= fatigueComponent->workDone;
            return energyComponent->green <= 0;
        };
        bool updateYellow(EnergyComponent* energyComponent, FatigueComponent* fatigueComponent) {
            if (energyComponent->yellow <= 0) {
                return false;
            }
            energyComponent->yellow -= fatigueComponent->workDone;
            return energyComponent->yellow <= 0;
        };
        bool updateRed(EnergyComponent* energyComponent, FatigueComponent* fatigueComponent) {
            if (energyComponent->red <= 0) {
                return false;
            }
            energyComponent->red-= fatigueComponent->workDone;
            return energyComponent->red <= 0;
        };
        bool updateBlack(EnergyComponent* energyComponent, FatigueComponent* fatigueComponent) {
            if (energyComponent->black <= 0) {
                return false;
            }
            energyComponent->black -= fatigueComponent->workDone;
            return energyComponent->black <= 0;
        }
};

struct CyclingSimulator {
    ComponentManager cm;
    EntityManager em;
    SystemManager sm;

    float dt = 0.1;
    LevelDetails level; 

    void init() {
        //register components
        cm.registerComponent<RiderComponent>();
        cm.registerComponent<NameComponent>();
        cm.registerComponent<EnergyComponent>();
        cm.registerComponent<SpeedComponent>();
        cm.registerComponent<DistanceComponent>();
        cm.registerComponent<FatigueComponent>();
        cm.registerComponent<LengthComponent>();
        cm.registerComponent<NextSegmentComponent>();
        cm.registerComponent<SegmentComponent>();

        //register systems, order is important (it is the update order)
        sm.registerSystem<SpeedSystem>(em, cm);
        sm.registerSystem<FatigueSystem>(em, cm);
        sm.registerSystem<SegmentSystem>(em, cm);
        
        //load level
        Entity firstSemgnet = createSegment("Dedinka vo Flandroch", 10000, 0);
        Entity krpa = createSegment("Krpal", 5000, 6);
        joinSegment(firstSemgnet, krpa);
        Entity downhill = createSegment("Serpentinky", 5000, -3);
        joinSegment(krpa, downhill);
        Entity flat = createSegment("Rovinecka", 15000, 0);
        joinSegment(downhill, flat);
        Entity kom = createSegment("Alp'duez", 17000, 12);
        joinSegment(flat, kom);
        
        //prepare riders
        createRider("Joonas", 69, 400, 2500000, firstSemgnet);
        createRider("Pogi", 66, 450, 2250000, firstSemgnet);
        createRider("MvP", 75, 400, 2000000, firstSemgnet);
        createRider("Remco", 73, 400, 1950000, firstSemgnet);
        // std::random_device rd;
        // std::mt19937 gen(rd());
        // std::uniform_int_distribution<> weight(60, 90);
        // std::uniform_int_distribution<> ftp(300, 470);
        // std::uniform_int_distribution<> energy(2500000, 3000000);
        // for (int i = 0; i < 10000; i++) {
        //     createRider("Rider"+std::to_string(i), weight(gen), ftp(gen), energy(gen), firstSemgnet);
        // }

        //init system views
        sm.updateComponentMask(em);
    }

    void run() {
        auto simulationStart = std::chrono::high_resolution_clock::now();
        while (!level.ridersOnRoute.empty()) {
            sm.update(dt, level);
            sm.updateComponentMask(em);
            //todo render
            level.raceTime += dt;
        }
        auto simulationStop = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = simulationStop - simulationStart;
        int i = 1;
        for (const auto& entity : level.classification) {
            auto& name = cm.getComponent<NameComponent>(entity);
            auto& energy = cm.getComponent<EnergyComponent>(entity);
            std::cout << i++ << ". place: " << name.name << " energy left: " << energy.green << "J" << std::endl;
        }
        std::cout << "Simulation took: " << duration.count() << "ms " << std::endl;
    }

    private: 
        Entity createSegment(std::string name, float length, float grade) {
            Entity segmentId = em.createEntity(NAME | LENGTH | SEGMENT);
            cm.addComponent<NameComponent>(segmentId, NameComponent(name));
            cm.addComponent<LengthComponent>(segmentId, LengthComponent(length));
            cm.addComponent<SegmentComponent>(segmentId, SegmentComponent(grade));

            level.routeLenght += length;
            return segmentId;
        }
        void joinSegment(Entity currentSegment, Entity nextSegment) {
            cm.addComponent<NextSegmentComponent>(currentSegment, NextSegmentComponent(nextSegment));
            em.entityComponentMask.at(currentSegment).nextFrame &= NEXT_SEGMENT;
        }

        void createRider(std::string name, float weight, float ftp, float energy, Entity firstSegment){
            Entity riderId = em.createEntity(NAME | RIDER | SPEED | DISTANCE | FATIGUE | ENERGY);
            cm.addComponent<NameComponent>(riderId, NameComponent(name));
            cm.addComponent<RiderComponent>(riderId, RiderComponent(weight, ftp));
            cm.addComponent<SpeedComponent>(riderId, SpeedComponent());
            LengthComponent& firstSegmentC = cm.getComponent<LengthComponent>(firstSegment);
            cm.addComponent<DistanceComponent>(riderId, DistanceComponent(firstSegment, firstSegmentC.length, level.routeLenght));
            cm.addComponent<FatigueComponent>(riderId, FatigueComponent(ftp));
            cm.addComponent<EnergyComponent>(riderId, EnergyComponent(ftp, energy));

            level.ridersOnRoute.emplace(riderId);
        }
};

int main(int argc, char const *argv[])
{
    CyclingSimulator simulator;
    
    try {
        simulator.init();
        simulator.run();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }    

    return EXIT_SUCCESS;
}
