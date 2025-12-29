/*
Save + Load settings for simplewalker
int and float conversions follow rules for std::stof() and std::stoi()
The following are equivalent ways to access nested nodes:
 - settings->cstr({"outer_block", "mid_block", "inner_block"})
 - settings->cstr("outer_block.mid_block.inner_block")   (not preferred, more overhead)
TODO:
    write settings to file
    default value
    remove (groupname, fieldname) overloads and use vector overloads instead
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
    const char *cstr(const std::vector<const char*> &fieldnames) const;
    float f(const char * const fieldname) const;
    float f(const char * const groupname, const char * const fieldname) const;
    float f(const std::vector<const char *> &fieldnames) const;
    int i(const char * const fieldname) const;
    int i(const char * const groupname, const char * const fieldname) const;
    int i(const std::vector<const char *> &fieldnames) const;
    bool b(const char * const fieldname) const;
    bool b(const char * const groupname, const char * const fieldname) const;
    bool b(const std::vector<const char *> &fieldnames) const;
    bool exists(const char * const fieldname) const;
    bool exists(const std::vector<const char *> fieldnames) const;
    std::vector<float> vf(const char * const fieldname) const;
    std::vector<float> vf(const char * const groupname, const char * const fieldname) const;
    const std::string filename;
};

#endif
