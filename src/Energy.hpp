#pragma once

class Energy {
    private:
        double green;
        double yellow;
        double red;
        double black;
    public:
        Energy(double total): green(total*0.55), yellow(total*0.25), red(total*0.15), black(total*0.05) {}
        void restUp(double total);
        bool updateGreen(const double workDone);
        bool updateYellow(const double workDone);
        bool updateRed(const double workDone);
        bool updateBlack(const double workDone);
};