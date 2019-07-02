#include "iniReader.h"

void iniReader::init(std::string ini_file){
    ini_settings = extract_settings(ini_file);
}

std::vector<std::string> iniReader::extract_lines(std::string ini_file){
    std::vector<std::string> lines;
    std::string line;
    std::ifstream file(ini_file);
    while(std::getline(file,line)){
        line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
        line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
        lines.push_back(line);
    }
    return (lines);
}

iniReader::settings iniReader::extract_settings(std::string ini_file){
    iniReader::settings set;
    iniReader::settingsGroup setGroup;
    std::vector<std::string> ini_file_lines = extract_lines(ini_file);
    int i = 0;
    for (std::vector<std::string>::iterator it = ini_file_lines.begin(); it != ini_file_lines.end(); ++it){
        std::string line = *it;
        if (line.at(0) == '[' && line.back() == ']'){
            //new group found
            if (i != 0){
                //store all vars to this point in previous group
                set.groups.push_back(setGroup);
            }
            //new group
            std::string groupName = line.substr(1, line.size()-2);
            iniReader::settingsGroup newGroup;
            setGroup = newGroup;
            setGroup.group = groupName;
        } else {
            iniReader::settingsVar setVar;
            int delimLoc = line.find("=");
            std::string varName = line.substr(0, delimLoc-1);
            std::string varVal = "";
            if (delimLoc + 2 < line.size()){
                varVal = line.substr(delimLoc+2, line.back());
            }
            setVar.val = varVal;
            setVar.var = varName;
            setGroup.vars.push_back(setVar);
        }
        if (i == ini_file_lines.size()-1){
            iniReader::settingsVar setVar;
            std::string varName = line;
            std::string varVal = line;
            setVar.val = varVal;
            setVar.var = varName;
            setGroup.vars.push_back(setVar);
            set.groups.push_back(setGroup);
        }
        i++;
    }
    return (set);
}

std::string iniReader::value(std::string groupName, std::string varName, std::string returnValIfEmpty){
    std::vector<iniReader::settingsGroup> groups = ini_settings.groups;
    for (std::vector<iniReader::settingsGroup>::iterator it_g = groups.begin(); it_g != groups.end(); ++it_g){
        iniReader::settingsGroup group = *it_g;
        if (group.group == groupName){
            std::vector<iniReader::settingsVar> vars = group.vars;
            for (std::vector<iniReader::settingsVar>::iterator it_v = vars.begin(); it_v != vars.end(); ++it_v){
                iniReader::settingsVar var = *it_v;
                if (var.var == varName){
                    if (var.val == "" || var.val == " "){
                        return returnValIfEmpty;
                    } else {
                        return var.val;
                    }
                }
            }
            return "variable not found in group";
        }
    }
    return "group not found";
}