#include "settings.hpp"
#include <fstream>
#include <iostream>
#include <cstring>
#include "rapidxml.hpp"
using std::string;


struct settings_impl {
    rapidxml::xml_document<> doc;
    rapidxml::xml_node<> *node = nullptr;
    std::vector<char> text;
};


WalkerSettings::WalkerSettings(const string &filename)
    : impl(std::make_unique<settings_impl>())
{
    std::ifstream inFile(filename);
    if (inFile.is_open()) {
        impl->text = std::vector<char>((std::istreambuf_iterator<char>(inFile)), std::istreambuf_iterator<char>());
        inFile.close();
        impl->text.push_back('\0');
        impl->doc.parse<rapidxml::parse_trim_whitespace>(&impl->text[0]);
        impl->node = impl->doc.first_node("settings");
        if (!impl->node) std::cerr<<"Couldn't find settings node in "<<filename<<std::endl;
    }
    else {
        throw std::system_error(errno, std::generic_category(), filename);
    }
}

const char *WalkerSettings::cstr(const char * const groupname, const char * const fieldname) const {
    if (!impl->node) return nullptr;
    rapidxml::xml_node<> *node = impl->node->first_node(groupname);
    if (node) {
        node = node->first_node(fieldname);
        if (node) return node->value();
    }
    std::cout<<"Couldn't load "<< groupname << "::" << fieldname <<std::endl;
    return nullptr;
}

const char *WalkerSettings::cstr(const char * const fieldname) const {
    if (!impl->node) return nullptr;
    rapidxml::xml_node<> *node = impl->node->first_node(fieldname);
    if (node) return node->value();
    std::cout<<"Couldn't load "<<fieldname<<std::endl;
    return nullptr;
}

float WalkerSettings::f(const char * const fieldname) const {
    if (!impl->node) return 0.0;
    return std::atof( cstr(fieldname));
}
float WalkerSettings::f(const char * const groupname, const char * const fieldname) const {
    if (!impl->node) return 0.0;
    return std::atof( cstr(groupname, fieldname));
}

int WalkerSettings::i(const char * const fieldname) const {
    if (!impl->node) return 0;
    return std::atoi( cstr(fieldname));
}
int WalkerSettings::i(const char * const groupname, const char * const fieldname) const {
    if (!impl->node) return 0;
    return std::atoi( cstr(groupname, fieldname));
}

bool WalkerSettings::b(const char * const fieldname) const {
    const char * text = cstr(fieldname);
    return (text && std::strlen(text) > 0 && string("false").compare(text) != 0 && string("0").compare(text) != 0);
}
bool WalkerSettings::b(const char * const groupname, const char * const fieldname) const {
    const char * text = cstr(groupname, fieldname);
    return (text && std::strlen(text) > 0 && string("false").compare(text) != 0 && string("0").compare(text) != 0);
}

std::vector<float> strtovf(const char * text) {
    std::vector<float> out;
    if (!text) {return out;}
    char *after;
    while (std::strlen(text) > 0) {
        out.push_back(std::strtof(text, &after));
        text = after;
    }
    return out;
}

std::vector<float> WalkerSettings::vf(const char * const fieldname) const {
    return strtovf(cstr(fieldname));
}
std::vector<float> WalkerSettings::vf(const char * const groupname, const char * const fieldname) const {
    return strtovf(cstr(groupname, fieldname));
}

WalkerSettings::~WalkerSettings() = default;
