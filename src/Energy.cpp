#include <Energy.hpp>

void Energy::restore(double total) {
    green += total*0.55;
    yellow += total*0.25;
    red += total*0.15;
    black += total*0.05;
}


void Energy::eatShit(double intakeInCal) {
    double intakeInJ = intakeInCal * calInJ;
    restore(intakeInJ);
}

bool Energy::updateGreen(const double workDone) {
    if (green <= 0) {
        return false;
    }
    green -= workDone;
    return green <= 0;
};
bool Energy::updateYellow(const double workDone) {
    if (yellow <= 0) {
        return false;
    }
    yellow -= workDone;
    return yellow <= 0;
};
bool Energy::updateRed(const double workDone) {
    if (red <= 0) {
        return false;
    }
    red -= workDone;
    return red <= 0;
};
bool Energy::updateBlack(const double workDone) {
    if (black <= 0) {
        return false;
    }
    black -= workDone;
    return black <= 0;
}
//todo minOf(color, 0)
double Energy::getGreen() {
    return green;
}
double Energy::getYellow() {
    return yellow;
}

double Energy::getRed() {
    return red;
}

double Energy::getBlack() {
    return black;
}

bool Energy::anyDepleted() {
    return green <= 0 || yellow <= 0 || red <= 0 || black <= 0;
}