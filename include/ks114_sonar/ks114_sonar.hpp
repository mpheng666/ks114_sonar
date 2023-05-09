#ifndef KS114_SONAR_SONAR_HPP_
#define KS114_SONAR_SONAR_HPP_

#include <algorithm>
#include <iostream>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
namespace ks114_sonar
{
    struct SonarConfiguration
    {
        uint8_t firmware_version {};
        uint8_t manufacturing_date {};
        int baud_rate {};
        uint8_t address {};
        std::string noise_suppression_level {};
        std::string beam_angle {};
        uint8_t error_code {};

        void printConfig()
        {
            std::cout << "Baud_rate               : " << baud_rate << "\n";
            std::cout << "Address                 : " << std::hex << unsigned(address)
                      << "\n";
            std::cout << "Noise_suppression_level : " << noise_suppression_level << "\n";
            std::cout << "Beam_angle              : " << beam_angle << "\n";
        }
    };

    static const std::unordered_map<int, uint8_t> SONAR_ADDRESSES {
        {1, 0XD0},  {2, 0XD2},  {3, 0XD4},  {4, 0XD6},  {5, 0XD8},
        {6, 0XDA},  {7, 0XDC},  {8, 0XDE},  {9, 0XE0},  {10, 0XE2},
        {11, 0XE4}, {12, 0XE6}, {13, 0XE8}, {14, 0XEA}, {15, 0XEC},
        {16, 0XEE}, {17, 0XF8}, {18, 0XFA}, {19, 0XFC}, {20, 0XFE}};

    static const std::map<uint8_t, std::string> NOISE_SUPPRESSIONS {
        {0X70, "LEVEL_0_BATTERY_POWERED"},
        {0X71, "LEVEL_1_BATTERY_POWERED_DEFAULT"},
        {0X72, "LEVEL_2_USB_POWERED"},
        {0X73, "LEVEL_4_USB_POWERED_LONG_DISTANCE"},
        {0X74, "LEVEL_5_SWITCHING_POWERED"},
        {0X75, "LEVEL_6_SWITCHING_POWERED"}};

    static const std::map<uint8_t, std::string> BEAM_ANGLES {{0X7A, "MODE_0"},
                                                             {0X7B, "MODE_1_DEFAULT"},
                                                             {0X7C, "MODE_2"},
                                                             {0X7D, "MODE_3"},
                                                             {0X7E, "MODE_4"},
                                                             {0X80, "H100_V50"},
                                                             {0X81, "H90_V40"},
                                                             {0X82, "H80_V35"}};


    enum class DetectionMode
    {
        Far,
        Fast
    };

    const std::map<DetectionMode, uint8_t> DETECTION_MODE {{DetectionMode::Far, 0xB0},
                                                           {DetectionMode::Fast, 0x0F}};

    class Ks114Sonar
    {
        public:
        Ks114Sonar(int index);
        ~Ks114Sonar();
        std::vector<uint8_t> getSenseCommand(DetectionMode mode);
        std::vector<uint8_t> getConfigCommand();
        std::optional<double> decodeDistance(const std::vector<uint8_t>& data);
        std::optional<SonarConfiguration> decodeConfig(const std::vector<uint8_t>& data);
        
        SonarConfiguration sonar_config;
     
        private:
        int sonar_index_ {0};
        static constexpr uint8_t REGISTER_NUM_ {0x02};
        static constexpr uint8_t CONFIG_CMD_ {0x99};
        static constexpr size_t DISTANCE_RAW_BYTE_SIZE_ {2};
        static constexpr size_t CONFIG_RAW_BYTE_SIZE_ {22};
    };

}  // namespace ks114_sonar

#endif