#include <Energy.hpp>

void Energy::restUp(double total) {
    green = total*0.55;
    yellow = total*0.25;
    red = total*0.15;
    black = total*0.05;
};

bool Energy::updateGreen(const double workDone) {
    green -= workDone;
    return green <= 0;
};
bool Energy::updateYellow(const double workDone) {
    yellow -= workDone;
    return yellow <= 0;
};
bool Energy::updateRed(const double workDone) {
    red -= workDone;
    return red <= 0;
};
bool Energy::updateBlack(const double workDone) {
    black -= workDone;
    return black <= 0;
}