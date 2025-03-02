#include <Energy.hpp>
#include <Rider.hpp>
#include <Route.hpp>
#include <Segment.hpp>
#include <Team.hpp>

int main(int argc, char const *argv[]) {
    Team jumbo = Team("Jumbo visma");
    jumbo.addRider(Rider(1,"Joonas", 69, 600, 2500000));
    jumbo.addRider(Rider(2,"Sepp Kuss", 62, 290, 10, 4));

    Team redbull = Team("Redbull");
    redbull.addRider(Rider(3,"Roglic", 65, 300, 6000));
    redbull.addRider(Rider(4,"Wout van Aert", 70, 310, 600, 6));

    Segment easyStart = Segment(0, 1000);
    Segment slightlyUp = Segment(1, 3000);
    Segment backDown = Segment(5, 3000);
    Segment brutalStena = Segment(19, 3000);

    Route alpineEtape = Route();
    alpineEtape.addSegment(easyStart);
    alpineEtape.addSegment(slightlyUp);
    alpineEtape.addSegment(backDown);
    alpineEtape.addSegment(brutalStena);

    std::vector<Team> teams = {jumbo, redbull};
    alpineEtape.simulateRoute(teams);
    return 0;
}
