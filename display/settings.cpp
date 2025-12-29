#include "settings.hpp"
#include <fstream>
#include <iostream>
#include <cstring>
#include "rapidxml.hpp"  // https://rapidxml.sourceforge.net/manual.html


struct settings_impl {
    rapidxml::xml_document<> doc;  // allocator and document parent node
    rapidxml::xml_node<> *node = nullptr; // settings base node (without boilerplate)
    std::vector<char> text; // parsed xml document text
};


WalkerSettings::WalkerSettings(const std::string &_filename)
    : impl(std::make_unique<settings_impl>()), filename(_filename)
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

const char *missing_field(const std::vector<const char *> &fieldnames, unsigned int depth) {
    std::cerr << "Couldn't load {";
    for (unsigned i = 0; i < depth + 1; i++)
        std::cerr << fieldnames[i] << " ";
    std::cerr << "}" << std::endl;
    return nullptr;
}

const char *find_field(const std::vector<const char *> &fieldnames, rapidxml::xml_node<> *base_node, unsigned depth) {
    rapidxml::xml_node<> *node = base_node->first_node(fieldnames[depth]);
    if (!node) {
        return missing_field(fieldnames, depth);
    }
    if (fieldnames.size() == depth + 1) { // last iteration
        return node->value();
    }
    return find_field(fieldnames, node, depth + 1);
}

const char *WalkerSettings::cstr(const std::vector<const char *> &fieldnames) const {
    if (fieldnames.empty()) return nullptr;
    if (!impl->node) return nullptr;
    return find_field(fieldnames, impl->node, 0);
}

const char *WalkerSettings::cstr(const char * const groupname, const char * const fieldname) const {
    if (!impl->node) return nullptr;
    return cstr({groupname, fieldname});
}

const char *find_field(const char * const fieldname, rapidxml::xml_node<> *base_node) {
    rapidxml::xml_node<> *node = base_node->first_node(fieldname);
    if (node) return node->value();
    // check for '.' in fieldname, if so split and call recursively
    const char *dot_ptr = strchr(fieldname, '.');
    if (!dot_ptr) {  // if there's no '.' in fieldname
        std::cerr << "Couldn't load " << fieldname << std::endl;
        return nullptr;
    }
    size_t dot_index = dot_ptr - fieldname;
    char *first_fieldname = (char *)malloc(dot_index + 1);
    strncpy(first_fieldname, fieldname, dot_index);
    first_fieldname[dot_index] = '\0';
    node = base_node->first_node(first_fieldname);
    free(first_fieldname);
    if (node)
        return find_field(fieldname + dot_index + 1, node);
    std::cerr<<"Couldn't load " << fieldname << std::endl;
    return nullptr;
}

const char *WalkerSettings::cstr(const char * const fieldname) const {
    if (!impl->node) return nullptr;
    // TODO: try find string without dot here, if that doesn't work, copy fieldname string so find_field can modify it?
    return find_field(fieldname, impl->node);
}

float WalkerSettings::f(const char * const fieldname) const {
    if (!impl->node) return 0.0;
    return std::stof( cstr(fieldname));
}
float WalkerSettings::f(const char * const groupname, const char * const fieldname) const {
    if (!impl->node) return 0.0;
    return std::stof( cstr(groupname, fieldname));
}
float WalkerSettings::f(const std::vector<const char *> &fieldnames) const {
    if (!impl->node) return 0.0;
    return std::stof( cstr(fieldnames));
}

int WalkerSettings::i(const char * const fieldname) const {
    if (!impl->node) return 0;
    return std::stoi( cstr(fieldname));
}
int WalkerSettings::i(const char * const groupname, const char * const fieldname) const {
    if (!impl->node) return 0;
    return std::stoi( cstr(groupname, fieldname));
}
int WalkerSettings::i(const std::vector<const char *> &fieldnames) const {
    if (!impl->node) return 0;
    return std::stoi( cstr(fieldnames));
}

bool WalkerSettings::b(const char * const fieldname) const {
    const char * text = cstr(fieldname);
    if (!text) return false;
    std::string str { text };
    return (!str.empty() && str != "false" && str != "False" && str != "0");
}
bool WalkerSettings::b(const char * const groupname, const char * const fieldname) const {
    const char * text = cstr(groupname, fieldname);
    if (!text) return false;
    std::string str { text };
    return (!str.empty() && str != "false" && str != "False" && str != "0");
}
bool WalkerSettings::b(const std::vector<const char *> &fieldnames) const {
    const char * text = cstr(fieldnames);
    if (!text) return false;
    std::string str { text };
    return (!str.empty() && str != "false" && str != "False" && str != "0");
}

bool WalkerSettings::exists(const char * const fieldname) const {
    return (bool)cstr(fieldname);
}
bool WalkerSettings::exists(const std::vector<const char *> fieldnames) const {
    return (bool)cstr(fieldnames);
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
