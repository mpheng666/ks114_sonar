#include "ks114_sonar_utility/ks114_sonar_utility.hpp"

namespace ks114_sonar_utility {

KS114SonarUtility::KS114SonarUtility() {}

KS114SonarUtility::~KS114SonarUtility() {}

void KS114SonarUtility::start() {
    std::cout << "PLEASE MAKE SURE ONLY ONE SONAR IS CONNECTED DURING THIS "
                 "SESSION! \nDO NOT REMOVE THE SONAR UNITL THIS SESSION IS "
                 "CLOSED! \n";
    std::array<ks114_sonar::Ks114Sonar, 20> sonars;

    for (auto i = 0; i < sonars.size(); ++i) {
        sonars.at(i).setIndex(i + 1);
        if (sonars.at(i).start()) {
            std::cout << "Sonar connected: " << i + 1 << "\n";
            sonars.at(i).getSonarConfig().printConfig();
            break;
        }
    }
    selectMode();
}

bool KS114SonarUtility::selectMode() {
    bool sucess{false};
    std::cout << "Please select a mode to continue: \n 1. ADDRESS_MODIFICATION "
                 "\n 2. BAUTRATE_MODIFICATION \n 3. "
                 "NOISE_SUPPRESSION_MODIFICATION \n 4. BEAM_ANGLE_MODIFICATION "
                 "\n 5. DETECTION \n";

    int mode = -1;
    while (mode == -1) {
        int user_input;
        std::cin >> user_input;
        if (user_input >= 1 && user_input <= 5 && std::cin.good()) {
            mode = user_input;
        } else {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Please select number (1~5) \n";
        }
    }
    UMode_ = static_cast<UtilityMode>(mode);
    sucess = true;
    return sucess;
}

// bool KS114SonarUtility::openSerial(const std::string &serial_port,
//                                    const int baudrate) {
//     try {
//         Serial_.setPort(serial_port);
//         Serial_.setBaudrate(baudrate);
//         serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
//         Serial_.setTimeout(timeout);
//         Serial_.open();
//         if (Serial_.isOpen()) {
//             std::cout << "Serial port " << serial_port_
//                       << " is opened at baud rate " << baud_rate_ << '\n';
//             return true;
//         } else {
//             std::cerr << "Serial port " << serial_port_
//                       << " failed to open at baud rate " << baud_rate_ <<
//                       '\n';
//             return false;
//         }
//     } catch (const std::exception &e) {
//         std::cerr << e.what() << '\n';
//         Serial_.close();
//         return false;
//     }
//     return false;
// }

// bool KS114SonarUtility::getConnectedSensorInfo(uint8_t &output_address) {
//     bool sucess = false;
//     for (auto i = 0; i < SONAR_ADDRESSES_.size(); ++i) {
//         std::vector<uint8_t> byte_received;
//         uint8_t info_cmd[3]{SONAR_ADDRESSES_.at(i), 0x20, 0x99};
//         Serial_.write(info_cmd, 3);
//         std::this_thread::sleep_for(std::chrono::microseconds(50));
//         Serial_.flush();
//         if (Serial_.available()) {
//             Serial_.read(byte_received, 22);
//             output_address = byte_received.at(5);
//             std::cout << "Found sensor " << i << "connected!";
//             sucess = true;
//             break;
//         }
//     }
//     return sucess;
// }

// void KS114SonarUtility::runStateMachine(UtilityMode &utility_mode) {
//     switch (utility_mode) {
//     case UtilityMode::ADDRESS_MODIFICATION: {
//         this->setNewAddress(connected_sonar_);
//         break;
//     }
//     case UtilityMode::BAUTRATE_MODIFICATION: {
//         this->setNewBaudRate();
//         break;
//     }
//     case UtilityMode::NOISE_SUPPRESSION_MODIFICATION: {
//         this->setNewNoiseSuppressionLevel();
//         break;
//     }
//     case UtilityMode::BEAM_ANGLE_MODIFICATION: {
//         this->setNewBeamAngleMode();
//         break;
//     }
//     case UtilityMode::DETECTION: {
//         this->detect();
//         break;
//     }
//     default:
//         break;
//     }
//     return;
// }

// bool KS114SonarUtility::compareAddress(const uint8_t
// reference_sensor_address) {
//     bool success = false;
//     uint8_t connected_sensor_now;
//     if (this->getConnectedSensorInfo(connected_sensor_now)) {
//         if (connected_sensor_now == reference_sensor_address) {
//             return true;
//         }
//     } else {
//     }
//     return success;
// }

// bool KS114SonarUtility::setNewAddress(const uint8_t sensor_current_address) {
//     bool sucess = false;

//     std::cout << "Selected mode "
//               << "ADDRESS_MODIFICATION" << '\n';
//     std::cout << "Counterclockwise: Front(0,1), Side_left(2,3), Back(4,5), "
//                  "Side_right(6,7) \n";

//     std::cout << "Please insert new number! (0~7) \n";

//     int new_index;

//     std::cin >> new_index;

//     uint8_t address_cmd_A[3] = {sensor_current_address, 0x02, 0x9a};
//     uint8_t address_cmd_B[3] = {sensor_current_address, 0x02, 0x92};
//     uint8_t address_cmd_C[3] = {sensor_current_address, 0x02, 0x9e};
//     uint8_t address_cmd_D[3] = {sensor_current_address, 0x02,
//                                 SONAR_ADDRESSES_[new_index]};

//     Serial_.write(address_cmd_A, 3);
//     std::this_thread::sleep_for(std::chrono::microseconds(50));
//     Serial_.write(address_cmd_B, 3);
//     std::this_thread::sleep_for(std::chrono::microseconds(50));
//     Serial_.write(address_cmd_C, 3);
//     std::this_thread::sleep_for(std::chrono::microseconds(50));
//     Serial_.write(address_cmd_D, 3);
//     std::this_thread::sleep_for(std::chrono::microseconds(2000));

//     if (compareAddress(SONAR_ADDRESSES_[new_index])) {
//         sucess = true;
//     }

//     return sucess;
// }

// bool KS114SonarUtility::setNewBaudRate(const int baudrate) {
//     bool sucess = false;
//     std::cout << "Selected mode "
//               << "BAUTRATE_MODIFICATION" << '\n';
//     std::cout << "NOT IMPLEMENTED YET! '\n";
//     return sucess;
// }

// bool KS114SonarUtility::setNewNoiseSuppressionLevel(const int level) {
//     bool sucess = false;
//     std::cout << "Selected mode "
//               << "NOISE_SUPPRESSION_MODIFICATION" << '\n';
//     std::cout << "NOT IMPLEMENTED YET! '\n";
//     return sucess;
// }

// bool KS114SonarUtility::setNewBeamAngleMode(const int mode) {
//     bool sucess = false;
//     std::cout << "Selected mode "
//               << "BEAM_ANGLE_MODIFICATION" << '\n';
//     std::cout << "NOT IMPLEMENTED YET! '\n";
//     return sucess;
// }

// bool KS114SonarUtility::detect(const int mode) {
//     bool sucess = false;
//     std::cout << "Selected mode "
//               << "ADDRESS_MODIFICATION" << '\n';
//     std::cout << "NOT IMPLEMENTED YET! '\n";
//     return sucess;
// }

} // namespace ks114_sonar_utility
