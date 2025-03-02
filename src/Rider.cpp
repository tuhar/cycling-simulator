#include <Rider.hpp>
#include <Segment.hpp>

double Rider::getPowerOutput(double distance) const {
    if (distance < 10) return maxPower;  // Start strong
    if (distance < 90) return ftp * 0.50;  // Steady pace
    return maxPower;                      // Sprint finish
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

void Rider::updateMaxPower() {
    if (energy.getGreen() <= 0) {
        maxPower = ftp * 0.15;
    } else if (energy.getYellow() <= 0) {
        maxPower = ftp * 0.55;
    } else if (energy.getRed() <= 0) {
        maxPower = ftp * 0.95;
    } else if (energy.getBlack() <= 0) {
        maxPower = ftp;
    }
    std::cout << "Rider's " << name << " max power lowered to " << maxPower << std::endl;
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
        updateMaxPower();
    }

    //todo move to AI agent
    if (energy.anyDepleted() && energyBars > 0) {        
        energy.eatShit(1200);
        maxPower = ftp * 1.1;        
        energyBars--;
        std::cout << name << " eating energy bar. Max power reset to " << maxPower << ". Energy bars remaining: " << energyBars << std::endl;
    }

    // std::cout << "Rider: " << name << " going " << speed << "ms is on sector with grade" << segment->grade  << std::endl;
    return distanceSegment >= segment->length;
}