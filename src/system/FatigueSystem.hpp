#pragma once

#include <component/Energy.hpp>
#include <component/Rider.hpp>
#include <ecs/ComponentManager.hpp>

bool updateBlack(EnergyComponent* energyComponent, FatigueComponent* fatigueComponent) {
  if (energyComponent->black <= 0) {
    return false;
  }
  energyComponent->black -= fatigueComponent->workDone;
  return energyComponent->black <= 0;
}

bool updateRed(EnergyComponent* energyComponent, FatigueComponent* fatigueComponent) {
  if (energyComponent->red <= 0) {
    return updateBlack(energyComponent, fatigueComponent);
  }
  energyComponent->red -= fatigueComponent->workDone;
  return energyComponent->red <= 0;
};

bool updateYellow(EnergyComponent* energyComponent, FatigueComponent* fatigueComponent) {
  if (energyComponent->yellow <= 0) {
    return updateRed(energyComponent, fatigueComponent);
  }
  energyComponent->yellow -= fatigueComponent->workDone;
  return energyComponent->yellow <= 0;
};

bool updateGreen(EnergyComponent* energyComponent, FatigueComponent* fatigueComponent) {
  if (energyComponent->green <= 0) {
    return updateYellow(energyComponent, fatigueComponent);
  }
  energyComponent->green -= fatigueComponent->workDone;
  return energyComponent->green <= 0;
};

bool updateRiderFatigue(FatigueComponent* fatigueComponent, EnergyComponent* energyComponent, RiderComponent* riderComponent) {
  if (fatigueComponent->powerOutput <= energyComponent->greenEffort) {
    return updateGreen(energyComponent, fatigueComponent);
  } else if (fatigueComponent->powerOutput > energyComponent->greenEffort && fatigueComponent->powerOutput <= energyComponent->redEffort) {
    return updateYellow(energyComponent, fatigueComponent);
  } else if (fatigueComponent->powerOutput > energyComponent->yellowEffort && fatigueComponent->powerOutput <= riderComponent->ftp) {
    return updateRed(energyComponent, fatigueComponent);
  } else {
    return updateBlack(energyComponent, fatigueComponent);
  }
}

void updateMaxPower(FatigueComponent* fatigueComponent, EnergyComponent* energyComponent, RiderComponent* riderComponent) {
  if (energyComponent->green <= 0 && !energyComponent->greenDone) {
    fatigueComponent->maxPower = riderComponent->ftp * 0.15;
    energyComponent->greenDone = true;
  } else if (energyComponent->yellow <= 0 && !energyComponent->yellowDone) {
    fatigueComponent->maxPower = riderComponent->ftp * 0.55;
    energyComponent->yellowDone = true;
  } else if (energyComponent->red <= 0 && !energyComponent->redDone) {
    fatigueComponent->maxPower = riderComponent->ftp * 0.95;
    energyComponent->redDone = true;
  } else if (energyComponent->black <= 0 && !energyComponent->blackDone) {
    fatigueComponent->maxPower = riderComponent->ftp;
    energyComponent->blackDone = true;
  }
}

void updateFatigue(ComponentManager& cm) {
  auto view = cm.view<EnergyComponent, FatigueComponent, RiderComponent>();
  for (size_t i = 0; i < view.denseArray.size(); i++) {
    EnergyComponent* energy = view.getComponent<EnergyComponent>()[i];
    FatigueComponent* fatigue = view.getComponent<FatigueComponent>()[i];
    RiderComponent* rider = view.getComponent<RiderComponent>()[i];
    if (updateRiderFatigue(fatigue, energy, rider)) {
      updateMaxPower(fatigue, energy, rider);
    };
  }
}