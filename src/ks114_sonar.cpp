#include "ks114_sonar/ks114_sonar.hpp"

namespace ks114_sonar
{
    Ks114Sonar::Ks114Sonar(int index)
        : sonar_index_(index)
    {
        sonar_config.address = SONAR_ADDRESSES.at(index);
        // std::cout << "Sonar index: " << sonar_index_ << " is created \n";
    }

    Ks114Sonar::~Ks114Sonar() { }

    std::vector<uint8_t> Ks114Sonar::getSenseCommand(DetectionMode mode)
    {
        std::vector<uint8_t> sense_command;

        sense_command.emplace_back(sonar_config.address);
        sense_command.emplace_back(REGISTER_NUM_);
        sense_command.emplace_back(DETECTION_MODE.at(mode));

        return sense_command;
    }

    std::vector<uint8_t> Ks114Sonar::getConfigCommand()
    {
        std::vector<uint8_t> config_command;

        config_command.emplace_back(sonar_config.address);
        config_command.emplace_back(REGISTER_NUM_);
        config_command.emplace_back(CONFIG_CMD_);

        return config_command;
    }

    std::array<std::array<uint8_t, 3>, 4>
    Ks114Sonar::getUpdateAddressCommand(int new_index)
    {
        std::array<std::array<uint8_t, 3>, 4> update_command;
        if (new_index < 1 || new_index > 20)
        {
            return update_command;
        }
        update_command.at(0) = {sonar_config.address, REGISTER_NUM_, 0X9a};
        update_command.at(1) = {sonar_config.address, REGISTER_NUM_, 0X92};
        update_command.at(2) = {sonar_config.address, REGISTER_NUM_, 0X9e};
        update_command.at(3) = {
            sonar_config.address, REGISTER_NUM_, SONAR_ADDRESSES.at(new_index)};

        return update_command;
    }

    std::optional<double> Ks114Sonar::decodeDistance(const std::vector<uint8_t>& data,
                                                     DetectionMode mode)
    {
        if (data.size() != DISTANCE_RAW_BYTE_SIZE_)
        {
            return std::nullopt;
        }
        double distance_m {0.0};
        bool success {false};
        const double CONVERSION_FACTOR = (mode == DetectionMode::Fast) ? 5800.0 : 1000.0;
        auto val                       = static_cast<uint16_t>(data[0] << 8 | data[1]);
        distance_m                     = static_cast<float>(val) / CONVERSION_FACTOR;
        return distance_m;
    }

    std::optional<SonarConfiguration>
    Ks114Sonar::decodeConfig(const std::vector<uint8_t>& data)
    {
        if (data.size() != CONFIG_RAW_BYTE_SIZE_)
        {
            std::cout << "WRONG CONFIG SIZE! "
                      << "Expected: " << CONFIG_RAW_BYTE_SIZE_
                      << " Received: " << data.size() << "\n";
            return std::nullopt;
        }

        SonarConfiguration config;
        config.firmware_version        = data.at(0);
        config.manufacturing_date      = data.at(1);
        config.baud_rate               = static_cast<int>(data.at(4));
        config.address                 = data.at(5);
        config.noise_suppression_level = NOISE_SUPPRESSIONS.at(data.at(6));
        config.beam_angle              = BEAM_ANGLES.at(data.at(7));
        config.error_code              = data.at(8);

        return config;
    }

}  // namespace ks114_sonar
