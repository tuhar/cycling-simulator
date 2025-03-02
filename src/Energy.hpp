#pragma once

const double calInJ = 4184;

class Energy {
    private:
        double green;
        double yellow;
        double red;
        double black;
    public:
        Energy(double total): green(total*0.55), yellow(total*0.25), red(total*0.15), black(total*0.05) {}
        void restore(double total);
        bool updateGreen(const double workDone);
        bool updateYellow(const double workDone);
        bool updateRed(const double workDone);
        bool updateBlack(const double workDone);

        double getGreen();
        double getYellow();
        double getRed();
        double getBlack();

        bool anyDepleted();
        void eatShit(double intakeInCal);
};