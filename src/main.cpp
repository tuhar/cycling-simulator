#include <vector>
#include <string>
#include <iostream>
#include <cmath>
#include <random>

const double g = 9.8067;
const double Cd = 0.63; //move to rider
const double A = 0.509; //move to rider
const double rho = 1.22601; // move to segment

struct Rider {
    u_int32_t id;

    u_int32_t energy; // in J
    double weight; //for now with all the gear

    std::string name;
    Rider(u_int32_t id, std::string name, double weight): id(id), name(name), weight(weight) {};

    double getPowerOutput(double distance) const {
        if (distance < 10) return 400;  // Start strong
        if (distance < 90) return 250;  // Steady pace
        return 600;                      // Sprint finish
    }
};

class Team{
    private:
        std::vector<Rider> riders;
    public:
        std::string name;
        Team(std::string name): name(name){};
        
        void addRider(Rider rider) {
            riders.push_back(rider);
        };

        const std::vector<Rider>& getRiders() {
            return riders;
        }
};

class Segment {
    private:
        double grade;
        double length;
        double roadQuality = 0.005; //lower = better
        double windSpeed = 0; //for now

        int minSpeed() const {
            if (grade == 0) {
                return 35;
            } else if (grade > 0 && grade <= 3) {
                return 30;
            } else if (grade > 3 && grade <= 5) {
                return 20;
            } else if (grade > 5 && grade <= 10) {
                return 15;
            } else {
                return 5;
            }
        }

        int maxSpeed() const {
            if (grade == 0) {
                return 50;
            } else if (grade > 0 && grade <= 3) {
                return 37;
            } else if (grade > 3 && grade <= 5) {
                return 25;
            } else if (grade > 5 && grade <= 10) {
                return 20;
            } else {
                return 15;
            }
        }

    public:
        Segment(double grade, double length): grade(grade), length(length) {};

        int getPelotonSpeed(std::mt19937& gen) const {
            std::uniform_int_distribution<> distr(minSpeed(), maxSpeed());
            return distr(gen);
        }

        double calculateSpeed(const Rider& rider, double power) const {
            double a = 0.5 * Cd * A * rho;
            double b = windSpeed * Cd * A * rho;
            double c = (g * rider.weight * (sin(atan(grade/100)) + roadQuality* cos(atan(grade/100)))) + (0.5*Cd* A * rho * pow(windSpeed, 2));
            double d = -power;

            double Q = (3*a*c - pow(b,2))/(9*pow(a,2));
            double R = (9*a*b*c - 27*pow(a, 2)*d - 2*pow(b,3))/(54*pow(a,3));
            double QR = sqrt(pow(Q, 3) + pow(R, 2));

            double S = cbrt(R + QR);
            double T = cbrt(R - QR);
            return S + T - (b/(3*a));
        };

        void depleteStamina(const Rider& rider, double speed) const {
            double Fg = g * (sin(atan(grade/100))) * rider.weight;
            double Fr = g * cos(atan(grade/100)) * rider.weight * roadQuality;
            double Fd = 0.5 * Cd * A * rho * pow((speed + windSpeed), 2.0);
            std::cout << "Fg: " << Fg << " Fr: " << Fr << " Fd: " << Fd << std::endl;
            double Fresist = Fg + Fr + Fd;  
        
            double work = Fresist * length;
            // 30 km/h = 30/3.6
            double power = Fresist * speed;
            // rider.energy -= energySpent;
            // double calculatedSpeed = 
            std::cout << "Energy needed for this segment: " << work << "J. Rider power needed: " << power << "W" << std::endl;
            std::cout << "Kontrolni otazka zni: speed from power = " << calculateSpeed(rider, power) * 3.6 << std::endl;
        }

        double simulateTrack(const Rider& rider, double dt, double initSpeed, double initTime) {
            double speed = initSpeed;
            double distance = 0;
            double time = initTime;

            double theta = atan(grade/100);
            double Fg = g * (sin(theta)) * rider.weight;
            double Fr = g * cos(theta) * rider.weight * roadQuality;
            while (distance < length) {
                double Fd = 0.5 * Cd * A * rho * pow((speed + windSpeed), 2.0);
                double Fresist = Fg + Fr + Fd;
                double power = rider.getPowerOutput((distance/length) * 100);

                double acceleration = (power / (rider.weight * speed)) - (Fresist / rider.weight);
                std::cout << "Acceleration: " << acceleration << std::endl;
                speed += dt * acceleration;
                distance += speed * dt;
                time += dt;
                std::cout << "Distance: " << distance << "m. Speed: " << speed*3.6 << "km/h. Rider power: " << power << "W" << std::endl;
            }
            return time;
        }

        const double& getGrade() const { return grade; }
};

class Route {
    private:
        std::vector<Segment> segments;
    public:
        void addSegment(Segment segment) {
            segments.push_back(segment);
        };
        const std::vector<Segment>& getSegments() {
            return segments;
        };
};


int main(int argc, char const *argv[])
{
    std::random_device rd; // obtain a random number from hardware
    std::mt19937 gen(rd()); // seed the generator
    
    double initialSpeed = 0.1;
    double initialDistance = 0;
    double t = 0;
    const double dt = 0.1;

    Team jumbo = Team("Jumbo visma");
    Rider joonas = Rider(1,"Joonas", 60);
    jumbo.addRider(joonas);
    // jumbo.addRider(Rider(2,"Sepp Kuss", 62));

    Team redbull = Team("Redbull");
    // redbull.addRider(Rider(3,"Roglic", 65));
    // redbull.addRider(Rider(4,"Wout van Aert", 70));

    std::vector<Team> teams = {jumbo, redbull};

    std::cout << "Teams competing: " << std::endl;
    int i = 1;
    for (auto team = teams.begin(); team != teams.end(); team++) {
        
        std::cout << "\t" << i++ << ") " << team->name << std::endl;
        for(auto rider = team->getRiders().begin(); rider != team->getRiders().end(); rider++) {
            std::cout << "\t\t" << "- " << rider->id << rider->name << std::endl;
            // testSegment.depleteStamina(*rider, 8.33);
        }
    }

    Segment easyStart = Segment(0, 1000);
    Segment slightlyUp = Segment(1, 3000);
    Segment backDown = Segment(-1, 3000);
    Segment brutalStena = Segment(19, 3000);

    Route alpineEtape = Route();
    alpineEtape.addSegment(easyStart);
    // alpineEtape.addSegment(slightlyUp);
    // alpineEtape.addSegment(backDown);
    // alpineEtape.addSegment(brutalStena);

    std::cout << "Starting a race!" << std::endl;
    // for (auto segment = alpineEtape.getSegments().begin(); segment != alpineEtape.getSegments().end(); segment++) {
    //     double speed = segment->getPelotonSpeed(gen);
    //     std::cout << "We are on segment with grade " << segment->getGrade() << "%, peloton is riding at " << speed << "km/h" << std::endl;
    //     for (auto team = teams.begin(); team != teams.end(); team++) {
    //         for(auto rider = team->getRiders().begin(); rider != team->getRiders().end(); rider++) {
    //             // segment->depleteStamina(*rider, speed/3.6);
    //             segment->depleteStamina(*rider, 0);
    //         }
    //     }
    // }
    easyStart.simulateTrack(joonas, dt, initialSpeed, t);

    return 0;
}
