#include <glm/glm.hpp>
#include "SimpleIni.h"
#include <iostream>
#include <sstream> 
#include "Functions/parseString.h"

namespace Lobo {

inline void setValueVec3(CSimpleIniA* ini, const char* section, const char* value,
                  glm::vec3 v3) {
    std::ostringstream stringStream;
    stringStream << v3.x<<" " << v3.y <<" " << v3.z;
    std::string copyOfStr = stringStream.str();
    ini->SetValue(section, value, copyOfStr.c_str());
}

inline void getValueVec3(CSimpleIniA* ini, const char* section, const char* value,
                  glm::vec3 *v3)
{
    std::vector<float> load_buffer;
    std::string pszValue = ini->GetValue(section, value);
    Lobo::parseString<float>(pszValue,load_buffer);
    *v3 = glm::vec3(load_buffer[0],load_buffer[1],load_buffer[2]);
}

inline void setValuef(CSimpleIniA* ini, const char* section, const char* value,
                  float v) {
    std::string copyOfStr = std::to_string(v);
    ini->SetValue(section, value, copyOfStr.c_str());
}

inline void getValuef(CSimpleIniA* ini, const char* section, const char* value,
                  float* v) {
    std::string pszValue = ini->GetValue(section, value);
    *v = std::stof(pszValue);
}

}  // namespace Lobo