#include "ks114_sonar_utility/ks114_sonar_utility.hpp"

namespace ks114_sonar_utility {

KS114SonarUtility::KS114SonarUtility() {}

KS114SonarUtility::~KS114SonarUtility() {}

void KS114SonarUtility::start()
{
    std::cout << "PLEASE MAKE SURE ONLY ONE SONAR IS CONNECTED DURING THIS "
                 "SESSION! \nDO NOT REMOVE THE SONAR UNITL THIS SESSION IS "
                 "CLOSED! \n";

    for (auto i = 0; i < sonars_.size(); ++i) {
        sonars_.at(i).setIndex(i + 1);
    }
    for (auto i = 0; i < sonars_.size(); ++i) {
        if (sonars_.at(i).start()) {
            connected_sonar_index_ = i;
            std::cout << "Sonar connected: " << i + 1 << "\n";
            sonars_.at(i).getSonarConfig().printConfig();
            break;
        }
    }
    if (selectMode()) {
        runStateMachine();
    }
}

bool KS114SonarUtility::selectMode()
{
    bool success{false};
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
        }
        else {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Please select number (1~5) \n";
        }
    }
    UMode_ = static_cast<UtilityMode>(mode);
    success = true;
    return success;
}

void KS114SonarUtility::runStateMachine()
{
    switch (UMode_) {
    case UtilityMode::ADDRESS_MODIFICATION: {
        this->setNewAddress();
        break;
    }
    case UtilityMode::BAUTRATE_MODIFICATION: {
        this->setNewBaudRate();
        break;
    }
    case UtilityMode::NOISE_SUPPRESSION_MODIFICATION: {
        this->setNewNoiseSuppressionLevel();
        break;
    }
    case UtilityMode::BEAM_ANGLE_MODIFICATION: {
        this->setNewBeamAngleMode();
        break;
    }
    case UtilityMode::DETECTION: {
        double distance{};
        sonars_.at(connected_sonar_index_)
                .getDistance(ks114_sonar::DetectionMode::Fast, distance);
        std::cout << "Distance: " << distance << "\n";
        break;
    }
    default:
        break;
    }
    return;
}

bool KS114SonarUtility::setNewAddress()
{
    bool success = false;

    std::cout << "Selected mode "
              << "ADDRESS_MODIFICATION \n";

    std::cout << "\n";

    std::cout << "               FRONT                   \n"
              << "              _2___1_                  \n"
              << "           3 |       | 8               \n"
              << "             |       |                 \n"
              << "           4 |       | 7               \n"
              << "              -------                  \n"
              << "             5      6                  \n"
              << "               BACK                    \n"
              << "         sonar_index_1 = 0XD0;         \n"
              << "         sonar_index_2 = 0XD2;         \n"
              << "         array[0] = sonar_index_1      \n"
              << "         array[1] = sonar_index_2      \n";

    std::cout << "\n";
    std::cout << "Please insert a new index! (1~20) \n";

    int new_index = -1;

    while (new_index == -1) {
        int user_input;
        std::cin >> user_input;
        if (user_input >= 1 && user_input <= 20 && std::cin.good()) {
            new_index = user_input;
            uint8_t address_cmd_A[3] = {
                    sonars_.at(connected_sonar_index_).getSonarConfig().address,
                    0x02, 0x9a};
            uint8_t address_cmd_B[3] = {
                    sonars_.at(connected_sonar_index_).getSonarConfig().address,
                    0x02, 0x92};
            uint8_t address_cmd_C[3] = {
                    sonars_.at(connected_sonar_index_).getSonarConfig().address,
                    0x02, 0x9e};
            uint8_t address_cmd_D[3] = {
                    sonars_.at(connected_sonar_index_).getSonarConfig().address,
                    0x02, ks114_sonar::SONAR_ADDRESSES.at(new_index)};

            sonars_.at(connected_sonar_index_).sendSerialCmd(address_cmd_A);
            sonars_.at(connected_sonar_index_).sendSerialCmd(address_cmd_B);
            sonars_.at(connected_sonar_index_).sendSerialCmd(address_cmd_C);
            sonars_.at(connected_sonar_index_).sendSerialCmd(address_cmd_D);

            success = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));

            std::cout << "PLEASE RECONNECT THE POWER OF THE SONAR FOR IT AND "
                         "RUN THE SCRIPT AGAIN TO VERIFY THE NEW CONFIG! \n ";
        }
        else {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Please select number (1~20) \n";
        }
    }
    return success;
}

bool KS114SonarUtility::setNewBaudRate()
{
    bool success = false;
    std::cout << "Selected mode "
              << "BAUTRATE_MODIFICATION" << '\n';
    std::cout << "\n";

    std::cout << "NOT IMPLEMENTED YET! '\n";
    return success;
}

bool KS114SonarUtility::setNewNoiseSuppressionLevel()
{
    bool success = false;
    std::cout << "Selected mode "
              << "NOISE_SUPPRESSION_MODIFICATION" << '\n';
    std::cout << "\n";

    std::cout << "NOT IMPLEMENTED YET! '\n";
    return success;
}

bool KS114SonarUtility::setNewBeamAngleMode()
{
    bool success = false;
    std::cout << "Selected mode "
              << "BEAM_ANGLE_MODIFICATION" << '\n';
    int i = 1;
    for (const auto &[key, value] : ks114_sonar::BEAM_ANGLES) {
        std::cout << " " << i++ << ". " << value << "\n";
    }
    std::cout << "\n";
    std::cout << "The beam angles decrease with the mode increment \n";
    std::cout << "\n";

    std::cout << "Please insert a new mode! (1~8) \n";
    std::cout << "\n";
    int new_mode = -1;

    while (new_mode == -1) {
        int user_input;
        std::cin >> user_input;
        if (user_input >= 1 && user_input <= 8 && std::cin.good()) {
            new_mode = user_input;
            uint8_t address_cmd_A[3] = {
                    sonars_.at(connected_sonar_index_).getSonarConfig().address,
                    0x02, 0x9c};
            uint8_t address_cmd_B[3] = {
                    sonars_.at(connected_sonar_index_).getSonarConfig().address,
                    0x02, 0x95};
            uint8_t address_cmd_C[3] = {
                    sonars_.at(connected_sonar_index_).getSonarConfig().address,
                    0x02, 0x98};
            auto it = ks114_sonar::BEAM_ANGLES.begin();
            std::advance(it, new_mode - 1);
            uint8_t address_cmd_D[3] = {
                    sonars_.at(connected_sonar_index_).getSonarConfig().address,
                    0x02, it->first};

            sonars_.at(connected_sonar_index_).sendSerialCmd(address_cmd_A);
            sonars_.at(connected_sonar_index_).sendSerialCmd(address_cmd_B);
            sonars_.at(connected_sonar_index_).sendSerialCmd(address_cmd_C);
            sonars_.at(connected_sonar_index_).sendSerialCmd(address_cmd_D);

            success = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));

            std::cout << "PLEASE RECONNECT THE POWER OF THE SONAR FOR IT AND "
                         "RUN THE SCRIPT AGAIN TO VERIFY THE NEW CONFIG! \n ";
        }
        else {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Please select number (1~8) \n";
        }
    }
    return success;
}

} // namespace ks114_sonar_utility
