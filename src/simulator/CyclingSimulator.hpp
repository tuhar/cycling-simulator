#pragma once

#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

#include <ecs/ComponentManager.hpp>
#include <ecs/EntityManager.hpp>
#include <simulator/Level.hpp>

#include <system/SpeedSystem.hpp>
#include <system/FatigueSystem.hpp>
#include <system/SegmentSystem.hpp>
#include <system/RenderSystem.hpp>

#include <iostream>
#include <fstream>

#include <random>
#include <chrono>

struct CyclingSimulator {
    sf::RenderWindow window;
    sf::Font font;
    
    sf::View mainCamera;
    sf::View hudView;

    ComponentManager cm;
    EntityManager em;

    float dt = 0.01f;
    float simulationSpeed = 1.0f;

    float startElevation = 500.f;
    Level level;
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
                    updateSpeed(cm, dt, level);
                    accumulator -= dt;
                    level.raceTime += dt;
                }
                updateFatigue(cm);
                updateSegment(cm, level);
                
                window.clear(sf::Color::Black); 
                window.draw(routeMesh);
                render(window, cm, mainCamera, hudView);
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
            cm.emplace(riderId, DistanceComponent(firstSegment, firstLength.length, level.routeLenght, Physics::getFgravity(firstSegmentComponent.sinTheta, weight), Physics::getFrolling(firstSegmentComponent.cosTheta, firstSegmentComponent.roadQuality, weight)));
            cm.emplace(riderId, FatigueComponent(ftp));
            cm.emplace(riderId, EnergyComponent(ftp, energy));
            cm.emplace(riderId, PositionComponent({0.f, startingElevation}));

            sf::Text text(font);
            text.setPosition({0, y});
            text.setCharacterSize(12);

            sf::CircleShape sprite(20.f);
            sprite.setFillColor(color);
            sprite.setPosition(cm.get<PositionComponent>().get(riderId)->position);
            cm.emplace(riderId, RenderComponent(text, sprite));
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