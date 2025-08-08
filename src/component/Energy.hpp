#pragma once

struct EnergyComponent {
  float greenEffort;
  float yellowEffort;
  float redEffort;

  float green;
  float yellow;
  float red;
  float black;

  bool greenDone = false;
  bool yellowDone = false;
  bool redDone = false;
  bool blackDone = false;

  EnergyComponent(float ftp, float total)
      : greenEffort(ftp * 0.55),
        yellowEffort(ftp * 0.75),
        redEffort(ftp * 0.9),
        green(total * 0.55),
        yellow(total * 0.25),
        red(total * 0.15),
        black(total * 0.05) {}
};

struct FatigueComponent {
  float powerOutput = 0;
  float workDone = 0;
  float maxPower;

  FatigueComponent(float ftp) : maxPower(ftp * 1.1) {}
};