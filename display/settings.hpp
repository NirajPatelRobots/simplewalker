/*
Save + Load settings for simplewalker
TODO:
    fieldname with no groupname finds lowest level fieldname
    write settings to file
    unit tests
*/
#ifndef SIMPLEWALKER_SETTINGS
#define SIMPLEWALKER_SETTINGS
#include <vector>
#include <string>
#include <memory>

struct settings_impl;


class WalkerSettings {
    std::unique_ptr<settings_impl> impl;
public:
    WalkerSettings(const std::string &filename);
    ~WalkerSettings();
    const char *cstr(const char * const fieldname) const;
    const char *cstr(const char * const groupname, const char * const fieldname) const;
    float f(const char * const fieldname) const;
    float f(const char * const groupname, const char * const fieldname) const;
    int i(const char * const fieldname) const;
    int i(const char * const groupname, const char * const fieldname) const;
    bool b(const char * const fieldname) const;
    bool b(const char * const groupname, const char * const fieldname) const;
    std::vector<float> vf(const char * const fieldname) const;
    std::vector<float> vf(const char * const groupname, const char * const fieldname) const;
};

#endif
