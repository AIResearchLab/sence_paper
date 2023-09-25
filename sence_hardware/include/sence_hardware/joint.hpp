#pragma once

#include <string>

namespace sence_hardware {

struct Joint {
  int id;
  std::string name;
  double pos; //state
  double vel; //state
  double eff; //state
  double cmd; //position control
};

} // namespace sence_hardware