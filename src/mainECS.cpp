#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <cmath>
#include <functional>
#include <typeindex>
#include <memory>
#include <any>
#include <fmt/format.h>

#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

#include <random>
#include <chrono>

using Entity = u_int32_t;
struct LevelDetails {
    float routeLenght = 0;
    float raceTime = 0;
    std::vector<Entity> classification;
    std::unordered_set<Entity> ridersOnRoute;
    bool finished = false;
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
    float cosTheta = cos(theta);
    float sinTheta = sin(theta);

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
    sf::Vector2f velocity = {0,0};
};

struct PositionComponent {
    sf::Vector2f position;
    
    PositionComponent(sf::Vector2f position): position(position){};
};

struct DistanceComponent {
    Entity segment;
    double distanceRemainingOnSegment;
    double distanceRemainingTotal;
    double coveredDistance;
    double fGravity;
    double fRolling;
    
    DistanceComponent(Entity segment, double segmentLength, double totalLength, double fGravity, double fRolling): segment(segment), distanceRemainingOnSegment(segmentLength), distanceRemainingTotal(totalLength), coveredDistance(0.0), fGravity(fGravity), fRolling(fRolling) {};
};

struct FatigueComponent {
    float powerOutput = 0;
    float workDone = 0;
    float maxPower;

    FatigueComponent(float ftp): maxPower(ftp * 1.1){}
};

struct SpeedRenderComponent {
    sf::Text text;
    sf::CircleShape sprite;

    SpeedRenderComponent(sf::Text text, sf::CircleShape sprite): text(text), sprite(sprite) {}
};

float g = 9.8067;
float Cd = 0.63; //drag coefficient -> todo move to rider
float A = 0.509; //frontal area -> todo move to rider
float rho = 1.22601; //density of air -> todo move to segment

struct SparseSet {
    std::vector<size_t> sparseArray;
    std::vector<Entity> denseArray;

    bool contains(Entity e) {
        if (e >= sparseArray.size()) {
            return false;
        }
        return denseArray[sparseArray[e]] == e;
    }

    void add(Entity e) {
        size_t size = denseArray.size();
        denseArray.push_back(e);
        if (e >= sparseArray.size()) {
            sparseArray.resize(e + 1);  // use sentinel value
        }
        sparseArray[e] = size;
    }

    void remove(Entity e) {
        size_t lastIndex = denseArray.size() - 1;
        Entity lastEntity = denseArray[lastIndex];
        std::swap(denseArray[sparseArray[e]], denseArray[lastIndex]);
        std::swap(sparseArray[e], sparseArray[lastEntity]);
        denseArray.pop_back();
    }
};

template<typename Component>
using ComponentViewVector = std::vector<Component*>;
template<typename... Components>
using ComponentViewVectorTuple = std::tuple<ComponentViewVector<Components>...>;

template<typename... Components>
struct UpdateView: SparseSet {
    ComponentViewVectorTuple<Components...> components;

    void add(Entity entity, Components*... entityComponents) {
        SparseSet::add(entity);
        addComponents(std::index_sequence_for<Components...>{}, entityComponents...);
    }

    template<typename TargetComponent>
    ComponentViewVector<TargetComponent>& getComponent() {
        return std::get<ComponentViewVector<TargetComponent>>(components);
    }

    void remove(Entity entity) {
        if (!SparseSet::contains(entity)) return;
        size_t lastIndex = denseArray.size() - 1;
        Entity lastEntity = denseArray[lastIndex];
        removeComponents(sparseArray[entity], sparseArray[lastEntity], std::index_sequence_for<Components...>{});
        SparseSet::remove(entity);

        std::cout<< "View state: " << entity << std::endl;
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

template<typename Component>
struct Storage : SparseSet {

    std::vector<Component> components;

    void add(Entity e, Component&& c) {
        SparseSet::add(e);
        components.push_back(std::forward<Component>(c));
    }

    void remove(Entity e) {
        size_t lastIndex = components.size() - 1;
        std::swap(components[sparseArray[e]], components[lastIndex]);
        components.pop_back();
        SparseSet::remove(e);
    }

    Component* get(Entity e) {
        //todo safety checks
        return &components[sparseArray[e]];
    }
};

struct ComponentManager {
   std::unordered_map<std::type_index, std::any> entityComponents;
   std::unordered_map<std::type_index, std::unique_ptr<SparseSet>> betterComponents;

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

    template<typename Component>
    void emplace(Entity e, Component&& component) {
        std::type_index type = typeid(std::decay_t<Component>);
        auto it = betterComponents.find(type);
        if (it == betterComponents.end()) {
            betterComponents[type] = std::make_unique<Storage<Component>>();
        }
        Storage<Component>& storage = static_cast<Storage<Component>&>(*betterComponents[type]);
        storage.add(e, std::forward<Component>(component));
    }
    template<typename Component>
    void remove(Entity e) {
        std::type_index type = typeid(std::decay_t<Component>);
        auto it = betterComponents.find(type);
        if (it == betterComponents.end()) {
            return;
        }
        Storage<Component>& storage = static_cast<Storage<Component>&>(*betterComponents[type]);
        storage.remove(e);
    }

    template<typename Component>
    Storage<Component>& get() {
        std::type_index type = typeid(std::decay_t<Component>);
        auto it = betterComponents.find(type);
        if (it == betterComponents.end()) {
            throw std::runtime_error("Unkown component " + std::string(type.name()) + "!");
        }
        return static_cast<Storage<Component>&>(*betterComponents[type]);
    }

    template<typename... Components>
    UpdateView<Components...> view() {
        UpdateView<Components...> view;
        SparseSet* smallest = nullptr;

        (([&] {
            auto& storage = get<Components>();
            if (!smallest || storage.denseArray.size() < smallest->denseArray.size()) {
                smallest = &storage;
            }
        }()), ...);

        for(auto entity: smallest->denseArray) {
            bool hasAll = ((get<Components>().contains(entity)), ...);
            if (hasAll) {
                auto components = std::make_tuple(get<Components>().get(entity)...);
                std::apply([&](Components*... ptrs) {
                    view.add(entity, ptrs...);
                }, components);
            }
        }
        return view;
    }

    template<typename Component>
    bool hasComponent(Entity entity) {
        std::type_index type = typeid(Component);
        auto it = entityComponents.find(type);
        return it != entityComponents.end();
    }

};

struct EntityManager {
    //todo reusable entities
    Entity sequence = 0;

    Entity createEntity() {
        Entity entity = sequence++;
        return entity;
    }
};

struct SpeedSystem {
    static void update(ComponentManager& cm, float dt, LevelDetails& level) {
        auto view = cm.view<RiderComponent, DistanceComponent, SpeedComponent, FatigueComponent, PositionComponent>();
        auto& segmentStorage = cm.get<SegmentComponent>();
        auto* riders = view.getComponent<RiderComponent>().data();
        auto* distances = view.getComponent<DistanceComponent>().data();
        auto* speeds = view.getComponent<SpeedComponent>().data();
        auto* fatigues = view.getComponent<FatigueComponent>().data();
        auto* positions = view.getComponent<PositionComponent>().data();

        std::unordered_map<Entity, SegmentComponent*> segmentCache;
        
        for (size_t i = 0; i < view.denseArray.size(); i++) {
                RiderComponent* rider = riders[i];
                DistanceComponent* distance = distances[i];
                auto segIt = segmentCache.find(distance->segment);
                if (segIt == segmentCache.end()) {
                    segIt = segmentCache.emplace(distance->segment, segmentStorage.get(distance->segment)).first;
                }
                SegmentComponent* segment = segIt->second;
                SpeedComponent* speed = speeds[i];
                FatigueComponent* fatigue = fatigues[i];
                PositionComponent* position = positions[i];

                float completition = (distance->distanceRemainingTotal/level.routeLenght) * 100; //todo fix inverted distance
                float powerOutput = getPowerOutput(rider, fatigue, completition);
                float fResist = distance->fGravity + distance->fRolling + getFdrag(segment, speed);
                float acceleration = (powerOutput / (rider->weight * speed->speed)) - (fResist / rider->weight);
                speed->speed += dt * acceleration;
                speed->velocity.x = speed->speed * segment->cosTheta;
                speed->velocity.y = -1*speed->speed * segment->sinTheta;
                position->position+=speed->velocity * dt;

                //when more complex move to its own system
                double coveredDistance = speed->speed * dt;
                distance->distanceRemainingOnSegment -= coveredDistance;
                distance->distanceRemainingTotal -= coveredDistance;
                distance->coveredDistance = coveredDistance;

                fatigue->workDone = fResist * coveredDistance;
                fatigue->powerOutput = powerOutput;
        }
    }
    static float getFgravity(SegmentComponent& segment, uint16_t riderWeight) {
        return g * (sin(segment.theta)) * riderWeight;
    };
        
    static float getFrolling(SegmentComponent& segment, uint16_t riderWeight) {
        return g * cos(segment.theta) * riderWeight * segment.roadQuality;
    }
    private:         
        static float getPowerOutput(RiderComponent* rider, FatigueComponent* energyConsumedComponent, float completition) {
            if (completition > 90) return energyConsumedComponent->maxPower;  // Start strong
            if (completition > 10) return rider->ftp * 0.50;  // Steady pace
            return energyConsumedComponent->maxPower;                      // Sprint finish
        }

        
        static float getFdrag(SegmentComponent* segment, SpeedComponent* speedComponent) {
            return 0.5 * Cd * A * rho * pow((speedComponent->speed + segment->windSpeed), 2.0);
        } 
};

struct SegmentSystem {
    static void update(ComponentManager& cm, LevelDetails& level) {  
        auto view = cm.view<DistanceComponent, RiderComponent>();
        std::vector<Entity> garbage;
        for (size_t i = 0; i < view.denseArray.size(); i++) {
                Entity& entity = view.denseArray[i];
                DistanceComponent* distance = view.getComponent<DistanceComponent>()[i];
                if (distance->distanceRemainingTotal <= 0) {
                    distance->coveredDistance = 0;
                    garbage.emplace_back(entity);
                    level.classification.emplace_back(entity);
                    level.ridersOnRoute.erase(entity);
                } else if (distance->distanceRemainingOnSegment <= 0) {
                    NextSegmentComponent& nextSegmentComponent = *cm.get<NextSegmentComponent>().get(distance->segment);
                    LengthComponent& nextSegmentLength = *cm.get<LengthComponent>().get(nextSegmentComponent.nextSegment);
                    SegmentComponent& segmentC = *cm.get<SegmentComponent>().get(nextSegmentComponent.nextSegment);
                    RiderComponent* rider = view.getComponent<RiderComponent>()[i];
                    distance->distanceRemainingOnSegment += nextSegmentLength.length;
                    distance->segment = nextSegmentComponent.nextSegment;
                    distance->fGravity = SpeedSystem::getFgravity(segmentC, rider->weight);
                    distance->fRolling = SpeedSystem::getFrolling(segmentC, rider->weight);
                }
        }
        for (Entity entity: garbage) {
            cm.remove<DistanceComponent>(entity);
            cm.remove<SpeedComponent>(entity);
            cm.remove<SpeedRenderComponent>(entity);
            cm.remove<PositionComponent>(entity);
            cm.remove<RiderComponent>(entity);
        }
    }
};

struct FatigueSystem {

    static void update(ComponentManager& cm) {
        auto view = cm.view<EnergyComponent, FatigueComponent, RiderComponent>();
        for (size_t i = 0; i < view.denseArray.size(); i++) {
            EnergyComponent* energy = view.getComponent<EnergyComponent>()[i];
            FatigueComponent* fatigue = view.getComponent<FatigueComponent>()[i];
            RiderComponent* rider = view.getComponent<RiderComponent>()[i];
            if (updateFatigue(fatigue, energy, rider)) {
                updateMaxPower(fatigue, energy, rider);
            };
        }
    }

    private: 
        static void updateMaxPower(FatigueComponent* fatigueComponent, EnergyComponent* energyComponent, RiderComponent* riderComponent) {
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

        static bool updateFatigue(FatigueComponent* fatigueComponent, EnergyComponent* energyComponent, RiderComponent* riderComponent) {
            if (fatigueComponent->powerOutput <= energyComponent->greenEffort) {
                return updateGreen(energyComponent, fatigueComponent);
            } else if (fatigueComponent->powerOutput > energyComponent->greenEffort && fatigueComponent->powerOutput <= energyComponent->redEffort) {
                return updateYellow(energyComponent, fatigueComponent);
            } else if (fatigueComponent->powerOutput > energyComponent->yellowEffort && fatigueComponent->powerOutput <= riderComponent->ftp) {
                return updateRed(energyComponent, fatigueComponent);
            } else {
                return updateBlack(energyComponent, fatigueComponent);
            }
        }

        static bool updateGreen(EnergyComponent* energyComponent, FatigueComponent* fatigueComponent) {
            if (energyComponent->green <= 0) {
                return updateYellow(energyComponent, fatigueComponent);
            }
            energyComponent->green -= fatigueComponent->workDone;
            return energyComponent->green <= 0;
        };
        static bool updateYellow(EnergyComponent* energyComponent, FatigueComponent* fatigueComponent) {
            if (energyComponent->yellow <= 0) {
                return updateRed(energyComponent, fatigueComponent);
            }
            energyComponent->yellow -= fatigueComponent->workDone;
            return energyComponent->yellow <= 0;
        };
        static bool updateRed(EnergyComponent* energyComponent, FatigueComponent* fatigueComponent) {
            if (energyComponent->red <= 0) {
                return updateBlack(energyComponent, fatigueComponent);
            }
            energyComponent->red-= fatigueComponent->workDone;
            return energyComponent->red <= 0;
        };
        static bool updateBlack(EnergyComponent* energyComponent, FatigueComponent* fatigueComponent) {
            if (energyComponent->black <= 0) {
                return false;
            }
            energyComponent->black -= fatigueComponent->workDone;
            return energyComponent->black <= 0;
        }
};

struct RenderSystem {

    static void render(sf::RenderWindow& window, ComponentManager& cm, sf::View& camera, sf::View& hud) {
        auto view = cm.view<SpeedRenderComponent, SpeedComponent, PositionComponent>();
        auto& nameStorage = cm.get<NameComponent>();
        auto& fatigueStorage = cm.get<FatigueComponent>();
        auto& distanceStorage = cm.get<DistanceComponent>();

        window.setView(hud);
        for(size_t i= 0; i < view.denseArray.size(); i++) {
            Entity& entity = view.denseArray[i];
            SpeedRenderComponent* text = view.getComponent<SpeedRenderComponent>()[i];
            SpeedComponent* speed = view.getComponent<SpeedComponent>()[i];
            NameComponent& name = *nameStorage.get(entity);
            FatigueComponent& fatigue = *fatigueStorage.get(entity);
            DistanceComponent& distance = *distanceStorage.get(entity);
            fmt::memory_buffer buf;
            fmt::format_to(std::back_inserter(buf), "{} speed: {:.1f} km/h, power output: {:.3f}W, remaining: {:.1f} m, fgravity: {:.1f}, frolling: {:.1f}", name.name, speed->speed * 3.6f, fatigue.powerOutput, distance.distanceRemainingTotal, distance.fGravity, distance.fRolling);
            text->text.setString(std::string(buf.data(), buf.size()));
            window.draw(text->text);
        }

        window.setView(camera);
        for (size_t i = 0; i < view.denseArray.size(); i++) {
            Entity& entity = view.denseArray[i];  
            PositionComponent* position = view.getComponent<PositionComponent>()[i];
            SpeedRenderComponent* text = view.getComponent<SpeedRenderComponent>()[i];            
            text->sprite.setPosition(position->position);
            if (camera.getCenter().x < text->sprite.getPosition().x) {    
                camera.setCenter(position->position);
            }
            window.draw(text->sprite);
        }
    }
};

struct CyclingSimulator {
    sf::RenderWindow window;
    sf::Font font;
    
    sf::View mainCamera;
    sf::View hudView;

    ComponentManager cm;
    EntityManager em;
    RenderSystem rs;

    float dt = 0.01f;
    float simulationSpeed = 1.0f;

    float startElevation = 500.f;
    LevelDetails level;
    sf::VertexArray routeMesh;

    void init() {
        window.create(sf::VideoMode({800, 600}), "Cycling Simulator");
        
        mainCamera.setCenter({400.f, 300.f});
        mainCamera.setSize({1600.f, 1200.f});
        mainCamera.setViewport(sf::FloatRect({0.f, 0.f}, {1.f,1.f}));

        hudView.setCenter({400.f, 300.f});
        hudView.setSize({800.f, 600.f});
        hudView.setViewport(sf::FloatRect({0.f, 0.f}, {1.f,1.f}));

        window.setView(mainCamera);

        if (!font.openFromFile("../../fonts/Arial.ttf")){
            std::runtime_error("Could not load font from file!");
        };        
        Entity firstSemgnet = loadLevel("../../levels/komEtape.txt");
        
        
        //prepare riders
        createRider("Joonas", 69, 400, 2500000, firstSemgnet, font, 0, startElevation -40, sf::Color::Yellow);
        createRider("Pogi", 66, 450, 2250000, firstSemgnet, font, 20, startElevation - 40, sf::Color::White);
        createRider("MvP", 75, 400, 2000000, firstSemgnet, font, 40, startElevation - 40, sf::Color::Red);
        createRider("Remco", 73, 400, 1950000, firstSemgnet, font, 60, startElevation - 40, sf::Color::Green);     
        createRider("Roglic", 76, 450, 210000, firstSemgnet, font, 80, startElevation -40, sf::Color::Blue);
        // std::random_device rd;
        // std::mt19937 gen(rd());
        // std::uniform_int_distribution<> weight(60, 90);
        // std::uniform_int_distribution<> ftp(300, 470);
        // std::uniform_int_distribution<> energy(2500000, 3000000);
        // for (int i = 0; i < 1; i++) {
        //     createRider("Rider"+std::to_string(i), weight(gen), ftp(gen), energy(gen), firstSemgnet, font, (80 + i*20), startElevation -40, sf::Color::Cyan);
        // }
    }

    void run() {
        sf::Text text(font);

        auto currentTime = std::chrono::high_resolution_clock::now();
        double accumulator = 0.0;
        while(window.isOpen()) {            

                while(const std::optional event = window.pollEvent()) {
                    if (event->is<sf::Event::Closed>()){
                        window.close();
                    }
                    if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>()){
                        if (keyPressed->scancode == sf::Keyboard::Scan::Right) {
                            if (simulationSpeed < 100) {
                                simulationSpeed += 5.0f;
                            }
                        }
                        if (keyPressed->scancode == sf::Keyboard::Scan::Left) {
                            if (simulationSpeed > 1) {
                                simulationSpeed -= 5.0f;
                            }
                        }
                    }
                }

            
            if (!level.ridersOnRoute.empty()) { //todo - can be determined from Distance/Speed component after the refactor?
                auto newTime = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> frameTime = newTime - currentTime;
                currentTime = newTime;

                accumulator += frameTime.count() * simulationSpeed;
                // std::cout << "frametime: " << frameTime << ", accumulator " << accumulator << std::endl;
    
                while (accumulator >= dt) {  
                    SpeedSystem::update(cm, dt, level);
                    accumulator -= dt;
                    level.raceTime += dt;
                }
                FatigueSystem::update(cm);
                SegmentSystem::update(cm, level);
                
                window.clear(sf::Color::Black); 
                window.draw(routeMesh);
                rs.render(window, cm, mainCamera, hudView);
                window.display();
            }
            if (level.ridersOnRoute.empty() && !level.finished) {
                int i = 1;
                window.setView(hudView);
                text.setPosition(sf::Vector2f(0,0));
                for (const auto& entity : level.classification) {
                    auto& name = *cm.get<NameComponent>().get(entity);
                    auto& energy = *cm.get<EnergyComponent>().get(entity);
                    text.setString(std::to_string(i++) = ". place: " + name.name + " energy left: " + std::to_string(energy.green) + "J" + "\n");

                    text.setPosition(text.getPosition()+sf::Vector2f(0, i * 10));                    
                    window.draw(text);
                }
                text.setCharacterSize(16);
                window.draw(text);
                window.display();
                level.finished = true;
            }
        }
    }

    private: 
        Entity createSegment(std::string name, float length, float grade) {
            Entity segmentId = em.createEntity();
            cm.emplace<NameComponent>(segmentId, NameComponent(name));
            cm.emplace<LengthComponent>(segmentId, LengthComponent(length));
            cm.emplace<SegmentComponent>(segmentId, SegmentComponent(grade));

            level.routeLenght += length;
            return segmentId;
        }
        void joinSegment(Entity currentSegment, Entity nextSegment) {
            cm.emplace<NextSegmentComponent>(currentSegment, NextSegmentComponent(nextSegment));
        }

        void createRider(std::string name, float weight, float ftp, float energy, Entity firstSegment, sf::Font& font, float y, float startingElevation, sf::Color color){
            Entity riderId = em.createEntity();
            LengthComponent& firstLength = *cm.get<LengthComponent>().get(firstSegment);
            SegmentComponent& firstSegmentComponent = *cm.get<SegmentComponent>().get(firstSegment);

            level.ridersOnRoute.emplace(riderId);

            cm.emplace(riderId, NameComponent(name));
            cm.emplace(riderId, RiderComponent(weight, ftp));
            cm.emplace(riderId, SpeedComponent());
            cm.emplace(riderId, DistanceComponent(firstSegment, firstLength.length, level.routeLenght, SpeedSystem::getFgravity(firstSegmentComponent, weight), SpeedSystem::getFrolling(firstSegmentComponent, weight)));
            cm.emplace(riderId, FatigueComponent(ftp));
            cm.emplace(riderId, EnergyComponent(ftp, energy));
            cm.emplace(riderId, PositionComponent({0.f, startingElevation}));

            sf::Text text(font);
            text.setPosition({0, y});
            text.setCharacterSize(12);

            sf::CircleShape sprite(20.f);
            sprite.setFillColor(color);
            sprite.setPosition(cm.get<PositionComponent>().get(riderId)->position);
            cm.emplace(riderId, SpeedRenderComponent(text, sprite));
        }
        
        Entity loadLevel(const std::string& levelPath) {
            std::ifstream file(levelPath);
            std::string l;
            size_t lines = 0;
            while (std::getline(file, l)) {
                lines++;
            }
            size_t meshSize = 2+2*lines;
            routeMesh.setPrimitiveType(sf::PrimitiveType::TriangleStrip);
            routeMesh.resize(meshSize); //todo read segment count and init segments in one loop if possible

            
            float currentElevation = 500.f;
            float currentBedrock = 520.f;
            routeMesh[0].position = sf::Vector2f(0.f, currentElevation);
            routeMesh[0].color = sf::Color::Green;
            routeMesh[1].position = sf::Vector2f(0.f, currentBedrock);
            routeMesh[1].color = sf::Color::Red;

            file.clear();
            file.seekg(0, std::ios::beg);
            std::string line;
            std::vector<Entity> segments;
            int i = 2;
            float totalLength = 0;
            while(std::getline(file, line)) {
                std::istringstream segment(line);
                std::string part;
                std::vector<float> segmentParts;
                while(std::getline(segment, part, ' ')) {
                    segmentParts.push_back(std::stof(part));
                }
                float length = segmentParts[0];
                float grade = segmentParts[1]/100;
                segments.push_back(createSegment("Name", length, segmentParts[1])); //todo name the segments?
                float elevation = length * grade;
                currentElevation += -1 * elevation;
                if (currentElevation > currentBedrock) {
                    currentBedrock = currentElevation + 20.f;
                    for (size_t j = 1; j < i; j+=2){
                        routeMesh[j].position.y = currentBedrock; 
                    }                
                }
                float x = sqrt(length*length - elevation*elevation);
                
                routeMesh[i].position = sf::Vector2f(x + totalLength, currentElevation);
                // std::cout << std::format("{} elevation vertex [{:.3f}, {:.3f}]",i, x, currentElevation) << std::endl;
                routeMesh[i++].color = sf::Color::Green;
                routeMesh[i].position = sf::Vector2f(x + totalLength, currentBedrock);
                // std::cout << std::format("{} bedrock vertex [{:.3f},{:.3f}]",i, x, currentBedrock) << std::endl;
                routeMesh[i++].color = sf::Color::Red;
                totalLength += x;
            }

            i = 0;
            while (i < segments.size() -1) {
                joinSegment(segments.at(i), segments.at(i+1));
                i++;
            }

            return segments[0];
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
