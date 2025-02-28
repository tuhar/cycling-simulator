#include <Route.hpp>

void Route::addSegment(Segment segment) {
    segments.push_back(segment);
};

void Route::simulateRoute(std::vector<Team>& teams) {
    double time = 0;
    double distanceOverflow = 0;
    const double dt = 0.1;   
    
    std::vector<Rider> ridersOnRoute;
    for (auto team = teams.begin(); team != teams.end(); team++) {
        for (auto rider = team->riders.begin(); rider != team->riders.end(); rider++) {
            rider->segment = segments.begin();
            ridersOnRoute.push_back(*rider);
        }
    }

    std::cout << "Starting a race!" << std::endl;
    while (!ridersOnRoute.empty()) {
        for (auto rider = ridersOnRoute.begin(); rider != ridersOnRoute.end();) {
            if (rider->updateRider(time, dt)) {
                if (rider->segment != (segments.end()-1)) {
                    double overflow = rider->distanceSegment - rider->segment->length;
                    rider->nextSegment(overflow);  
                    rider++;                   
                } else {
                    std::cout << "Rider: " << rider->name << " finished the race in " << time << "s." << std::endl;
                    rider = ridersOnRoute.erase(rider);
                }
            } else {
                rider++;
            }
        }
        time += dt;
    }
    std::cout << "Race finished!" << std::endl;
}