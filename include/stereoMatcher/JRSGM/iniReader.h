#ifndef INIREADER_H
#define INIREADER_H

#include <string>
#include <vector>
#include <algorithm>
#include <sstream>
#include <iostream>
#include <fstream>

class iniReader {

    public:

        struct settingsVar {
            std::string var;
            std::string val;
        };
        struct settingsGroup {
            std::string group;
            std::vector<settingsVar> vars;
        };
        struct settings {
            std::vector<settingsGroup> groups;
        };
    
        iniReader(std::string ini_file){
            init(ini_file);
        }
        std::string value(std::string groupName, std::string varName, std::string returnValIfEmpty);

    private:
        std::string ini_filepath = "multigpusgm_param_test.param";
        iniReader::settings ini_settings;
        void init(std::string ini_file);
        std::vector<std::string> extract_lines(std::string ini_file);
        iniReader::settings extract_settings(std::string ini_file);
};

#endif // INIREADER_H