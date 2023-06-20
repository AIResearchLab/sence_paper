#pragma once

namespace sence_hardware
{
class SenceHardwareInterface
{
public:
  SenceHardwareInterface();
  ~SenceHardwareInterface();

  void init();
  void update();
  void shutdown();

private:
};

}  // namespace sence_hardware
