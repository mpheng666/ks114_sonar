#include "ks114_sonar/ks114_sonar_utility.hpp"

namespace ks114_sonar
{
    KS114SonarUtility::KS114SonarUtility(const std::string& port_name)
        : serial_port_(port_name)
        , comms_handler_(port_name, baud_rate_, 200, true)
    {
    }

    KS114SonarUtility::~KS114SonarUtility() { }

    void KS114SonarUtility::start()
    {
        while (!comms_handler_.isPortOpen())
        {
            comms_handler_.start();
        }

        while (selectMode() != 7)
        {
            runStateMachine();
            std::cout << "\n";
        }
    }

    int KS114SonarUtility::selectMode()
    {
        std::cout << "Please select a mode to continue: \n 1. ADDRESS_MODIFICATION "
                     "\n 2. BAUTRATE_MODIFICATION \n 3. "
                     "NOISE_SUPPRESSION_MODIFICATION \n 4. BEAM_ANGLE_MODIFICATION "
                     "\n 5. DETECTION \n 6. SEARCH_SONAR \n 7. QUIT \n";

        int mode = -1;
        while (mode == -1)
        {
            int user_input;
            std::cin >> user_input;
            if (user_input >= 1 && user_input <= 7 && std::cin.good())
            {
                mode = user_input;
            }
            else
            {
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                std::cout << "Please select number (1~7) \n";
            }
        }
        UMode_ = static_cast<UtilityMode>(mode);
        return mode;
    }

    void KS114SonarUtility::runStateMachine()
    {
        switch (UMode_)
        {
            case UtilityMode::ADDRESS_MODIFICATION:
            {
                setNewAddress();
                break;
            }
            case UtilityMode::BAUTRATE_MODIFICATION:
            {
                setNewBaudRate();
                break;
            }
            case UtilityMode::NOISE_SUPPRESSION_MODIFICATION:
            {
                setNewNoiseSuppressionLevel();
                break;
            }
            case UtilityMode::BEAM_ANGLE_MODIFICATION:
            {
                setNewBeamAngleMode();
                break;
            }
            case UtilityMode::DETECTION:
            {
                int input = -1;
                while (input < 1 || input > 20)
                {
                    std::cout << "Please select a sonar index from (1~20) \n";
                    std::cin >> input;
                    if (input > 0 && input <= 20)
                    {
                        connected_sonar_index_ = input;
                    }
                }
                Ks114Sonar sonar(connected_sonar_index_);
                SonarReader reader(sonar, comms_handler_);

                if (auto distance = reader.getDistance(DetectionMode::Fast))
                {
                    std::cout << "Distance: " << distance.value() << "\n";
                }
                break;
            }
            case UtilityMode::AUTO_SEARCH_SONAR:
            {
                for (int i = 1; i <= 20; ++i)
                {
                    Ks114Sonar sonar(i);
                    auto command = sonar.getConfigCommand();
                    comms_handler_.write(command);
                    auto read_data = comms_handler_.read(22);
                    if (auto result = sonar.decodeConfig(read_data))
                    {
                        std::cout << "Found sonar with index: " << std::dec << i
                                  << " at address " << std::hex
                                  << unsigned(result.value().address) << "\n";
                    }
                }
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
        int input    = -1;
        while (input < 1 || input > 20)
        {
            std::cout << "Please select a sonar index from (1~20) \n";
            std::cin >> input;
            if (input > 0 && input <= 20)
            {
                connected_sonar_index_ = input;
            }
        }

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

        while (new_index == -1)
        {
            int user_input;
            std::cin >> user_input;

            if (user_input >= 1 && user_input <= 20 && std::cin.good())
            {
                Ks114Sonar sonar(connected_sonar_index_);
                new_index     = user_input;
                auto commands = sonar.getUpdateAddressCommand(new_index);
                while (!comms_handler_.isPortOpen())
                {
                    comms_handler_.start();
                }
                for (int i = 0; i < commands.size(); ++i)
                {
                    comms_handler_.write({commands.at(i).begin(), commands.at(i).end()});
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));

                success = true;

                std::cout << "PLEASE RECONNECT THE POWER OF THE SONAR FOR IT AND "
                             "RUN THE SCRIPT AGAIN TO VERIFY THE NEW CONFIG! \n ";
            }
            else
            {
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
        // std::cout << "Selected mode "
        //           << "BAUTRATE_MODIFICATION" << '\n';
        // std::cout << "\n";

        std::cout << "NOT IMPLEMENTED YET! NO REASON TO '\n";
        return success;
    }

    bool KS114SonarUtility::setNewNoiseSuppressionLevel()
    {
        bool success = false;

        int input = -1;
        while (input < 1 || input > 20)
        {
            std::cout << "Please select a sonar index from (1~20) \n";
            std::cin >> input;
            if (input > 0 && input <= 20)
            {
                connected_sonar_index_ = input;
            }
        }

        Ks114Sonar sonar(connected_sonar_index_);
        while (!comms_handler_.isPortOpen())
        {
            comms_handler_.start();
        }
        auto command = sonar.getConfigCommand();
        comms_handler_.write(command);
        auto read_data = comms_handler_.read(22);
        if (auto result = sonar.decodeConfig(read_data))
        {
            result.value().printConfig();
        }
        else
        {
            std::cout << "Could not find sensor! \n";
        }

        std::cout << "Selected mode "
                  << "NOISE_SUPPRESSION_MODIFICATION" << '\n';

        int i = 1;
        for (const auto& [key, value] : ks114_sonar::NOISE_SUPPRESSIONS)
        {
            std::cout << " " << i++ << ". " << value << "\n";
        }
        std::cout << "The suppression increases with the mode increment \n";
        std::cout << "\n";

        std::cout << "Please insert a new mode! (1~8) \n";
        std::cout << "\n";
        int new_mode = -1;

        while (new_mode == -1)
        {
            int user_input;
            std::cin >> user_input;
            if (user_input >= 1 && user_input <= 8 && std::cin.good())
            {
                new_mode = user_input;
                auto it  = ks114_sonar::NOISE_SUPPRESSIONS.begin();
                std::advance(it, new_mode - 1);

                std::vector<std::array<uint8_t, 3>> commands(4);
                commands.at(0) = {SONAR_ADDRESSES.at(connected_sonar_index_), 0x02, 0x9C};
                commands.at(1) = {SONAR_ADDRESSES.at(connected_sonar_index_), 0x02, 0x95};
                commands.at(2) = {SONAR_ADDRESSES.at(connected_sonar_index_), 0x02, 0x98};
                commands.at(3) = {
                    SONAR_ADDRESSES.at(connected_sonar_index_), 0x02, it->first};

                for (int i = 0; i < commands.size(); ++i)
                {
                    comms_handler_.write({commands.at(i).begin(), commands.at(i).end()});
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                success = true;

                std::cout << "PLEASE RECONNECT THE POWER OF THE SONAR FOR IT AND "
                             "RUN THE SCRIPT AGAIN TO VERIFY THE NEW CONFIG! \n ";
            }
            else
            {
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                std::cout << "Please select number (1~6) \n";
            }
        }
        return success;
    }

    bool KS114SonarUtility::setNewBeamAngleMode()
    {
        bool success = false;

        int input = -1;
        while (input < 1 || input > 20)
        {
            std::cout << "Please select a sonar index from (1~20) \n";
            std::cin >> input;
            if (input > 0 && input <= 20)
            {
                connected_sonar_index_ = input;
            }
        }

        Ks114Sonar sonar(connected_sonar_index_);
        while (!comms_handler_.isPortOpen())
        {
            comms_handler_.start();
        }
        auto command = sonar.getConfigCommand();
        comms_handler_.write(command);
        auto read_data = comms_handler_.read(22);
        if (auto result = sonar.decodeConfig(read_data))
        {
            result.value().printConfig();
        }
        else
        {
            std::cout << "Could not find sensor! \n";
        }
        std::cout << "Selected mode "
                  << "BEAM_ANGLE_MODIFICATION" << '\n';

        int i = 1;
        for (const auto& [key, value] : ks114_sonar::BEAM_ANGLES)
        {
            std::cout << " " << i++ << ". " << value << "\n";
        }
        std::cout << "The beam angles decrease with the mode increment \n";
        std::cout << "\n";

        std::cout << "Please insert a new mode! (1~8) \n";
        std::cout << "\n";
        int new_mode = -1;

        while (new_mode == -1)
        {
            int user_input;
            std::cin >> user_input;
            if (user_input >= 1 && user_input <= 8 && std::cin.good())
            {
                new_mode = user_input;
                auto it  = ks114_sonar::BEAM_ANGLES.begin();
                std::advance(it, new_mode - 1);

                std::vector<std::array<uint8_t, 3>> commands(4);
                commands.at(0) = {SONAR_ADDRESSES.at(connected_sonar_index_), 0x02, 0x9C};
                commands.at(1) = {SONAR_ADDRESSES.at(connected_sonar_index_), 0x02, 0x95};
                commands.at(2) = {SONAR_ADDRESSES.at(connected_sonar_index_), 0x02, 0x98};
                commands.at(3) = {
                    SONAR_ADDRESSES.at(connected_sonar_index_), 0x02, it->first};

                for (int i = 0; i < commands.size(); ++i)
                {
                    comms_handler_.write({commands.at(i).begin(), commands.at(i).end()});
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                success = true;

                std::cout << "PLEASE RECONNECT THE POWER OF THE SONAR FOR IT AND "
                             "RUN THE SCRIPT AGAIN TO VERIFY THE NEW CONFIG! \n ";
            }
            else
            {
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                std::cout << "Please select number (1~6) \n";
            }
        }
        return success;
    }

}  // namespace ks114_sonar
