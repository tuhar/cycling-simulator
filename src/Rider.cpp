#include <Rider.hpp>
#include <Segment.hpp>

double Rider::getPowerOutput(double distance) const {
    if (distance < 10) return 400;  // Start strong
    if (distance < 90) return 250;  // Steady pace
    return 600;                      // Sprint finish
}

bool Rider::updateFatigue(const double workDone, const double powerOutput) {
    if (powerOutput <= greenEffort) {
        return energy.updateGreen(workDone);
    } else if (powerOutput > greenEffort && powerOutput <= yellowEffort) {
        return energy.updateYellow(workDone);
    } else if (powerOutput > yellowEffort && powerOutput <= redEffort) {
        return energy.updateRed(workDone);
    } else {
        return energy.updateBlack(workDone);
    }
}
    
void Rider::nextSegment(double distanceOverflow) {
    distanceSegment = distanceOverflow;        
    segment++;
}

bool Rider::updateRider(double time, double dt) {
    double Fresist = segment->getFresist(*this);
    double power = getPowerOutput((distanceTotal/segment->length) * 100);
    double acceleration = (power / (weight * speed)) - (Fresist / weight);
    
    // std::cout << "Acceleration: " << acceleration << std::endl;
    speed += dt * acceleration;
    double coveredDistance = speed * dt;
    distanceSegment += coveredDistance;
    distanceTotal += coveredDistance;
    double work = Fresist * coveredDistance;
    if (updateFatigue(work, power)) {
        // std::cout << "Prepalil si! max output must be lowered" << std::flush;
    }
    // std::cout << "Rider: " << name << " going " << speed << "ms is on sector with grade" << segment->grade  << std::endl;
    return distanceSegment >= segment->length;
}