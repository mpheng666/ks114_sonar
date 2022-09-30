#ifndef KS114_SONAR_UTILITY_HPP_
#define KS114_SONAR_UTILITY_HPP_

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <string_view>
#include <map>
#include <unordered_map>

#include <serial/serial.h>

// 0x69: Program version, stored in Register 0;
// 0xa9: Mark of manufacturing date, stored in Register 1;
// 0x79: Communication baud rate of serial port, stored in Register 4;
// 0xe8: I2C or serial port address, stored in Register 5;
// 0x71: Noise reduction level, stored in Register 6;
// 0x7b: Factory default setting for configuring dead zone. Stored in Register 7;
// 0xe3: Error code, stored in Register 8;
// 0x68: Initialization completion mark, stored in Register 9; its value is 0x69 when initialization starts;

namespace ks114_sonar
{
class Ks114Sonar
{
public:
  Ks114Sonar();
  ~Ks114Sonar();
  void start();
  // void getFirmwareVersion();
  // void getManufacturingDate();
  // void getBaudRate();
  // void getAddress();
  // void getNoiseSuppressionLevel();
  // void getBeamAngle();
  // void getErrorCode();

private:
  enum class PortState
  {
    PortUnopened,
    PortOpened,
    PortErrored
  };

  const std::map<uint8_t, int> SONAR_ADDRESSES_{ { 0XD0, 1 },  { 0XD2, 2 },  { 0XD4, 3 },  { 0XD6, 4 },  { 0XD8, 5 },
                                                 { 0XDA, 6 },  { 0XDC, 7 },  { 0XDE, 10 }, { 0XE0, 9 },  { 0XE2, 10 },
                                                 { 0XE4, 11 }, { 0XE6, 12 }, { 0XE8, 13 }, { 0XEA, 14 }, { 0XEC, 15 },
                                                 { 0XEE, 16 }, { 0XF8, 17 }, { 0XFA, 18 }, { 0XFC, 19 }, { 0XFE, 20 } };

  const std::map<uint8_t, std::string_view> NOISE_SUPPRESSIONS_{
    { 0X70, "LEVEL_0_BATTERY_POWERED" },   { 0X71, "LEVEL_1_BATTERY_POWERED_DEFAULT" },
    { 0X72, "LEVEL_2_USB_POWERED" },       { 0X73, "LEVEL_4_USB_POWERED_LONG_DISTANCE" },
    { 0X74, "LEVEL_5_SWITCHING_POWERED" }, { 0X75, "LEVEL_6_SWITCHING_POWERED" }
  };

  const std::map<uint8_t, std::string_view> BEAM_ANGLES_{ { 0X7A, "MODE_0" },  { 0X7B, "MODE_1_DEFAULT" },
                                                          { 0X7C, "MODE_2" },  { 0X7D, "MODE_3" },
                                                          { 0X7E, "MODE_4" },  { 0X80, "H100_V50" },
                                                          { 0X81, "H90_V40" }, { 0X82, "H80_V35" } };

struct 
  uint8_t firmware_version_;
  uint8_t manufacturing_date_;
  uint8_t baud_rate_;
  uint8_t address_;
  uint8_t noise_suppression_level_;
  uint8_t beam_angle_;
  uint8_t error_code_;

  std::string serial_port_{"/dev/ttyUSB0"};
  int baud_rate_{115200};
  serial::Serial Serial_;
  uint8_t connected_sonar_{0XE8};
  uint8_t current_noise_suppression_level_ {0X71};
  uint8_t current_beam_config_{0X7B};

  void getSonarInfo();
};
}  // namespace ks114_sonar

#endif