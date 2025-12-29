/* test WalkerSettings
TODO:
*/
#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>
#include "settings.hpp"


const std::string SAVE_FILENAME { "/tmp/unittest_settings_example.xml" };

const std::string example_settings_str = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
                                         "<settings>\n"
                                         "<outer_block>\n"
                                         "    <0.5> 0.5 </0.5>\n"
                                         "    <empty_block> </empty_block>\n"
                                         "    <text_block> Niraj </text_block>\n"
                                         "    <mid_block> \n"
                                         "        <inner_block> nested </inner_block>\n"
                                         "    </mid_block>\n"
                                         "</outer_block>\n"
                                         "<0.5> 0.5 </0.5>\n"
                                         "<.5> .5 </.5>\n"
                                         "<false> false </false>\n"
                                         "<False> False </False>\n"
                                         "<zero> 0 </zero>\n"
                                         "<reciprocals>\n"
                                         "    <0.5> 2 </0.5>\n"
                                         "</reciprocals>\n"
                                         "<array> 1 2 3 </array>\n"
                                         "</settings>\n";

class WalkerSettingsTest : public ::testing::Test {
protected:
    std::unique_ptr<WalkerSettings> settings;
    void SetUp() override {
        std::ofstream outFile(SAVE_FILENAME);
        if (!outFile.is_open()) {
            ASSERT_TRUE(false);
        }
        outFile.write(example_settings_str.c_str(), (long)(example_settings_str.length()));
        outFile.close();
        settings = std::make_unique<WalkerSettings>(SAVE_FILENAME);
    }
    void TearDown() override {
        std::filesystem::remove(SAVE_FILENAME);
    }
};


TEST_F(WalkerSettingsTest, OpenAndClose) {
    EXPECT_STREQ(SAVE_FILENAME.c_str(), settings->filename.c_str());
    settings.reset();
}

TEST_F(WalkerSettingsTest, Bad_Filename) {
    EXPECT_THROW({
            settings = std::make_unique<WalkerSettings>("bad filename doesn't exist.xml");
        }, std::system_error);
}

TEST_F(WalkerSettingsTest, Load_Int) {
    EXPECT_EQ(settings->i("reciprocals", "0.5"), 2);
}

TEST_F(WalkerSettingsTest, Load_Int_Empty) {
    EXPECT_THROW({
        settings->i("outer_block", "empty_block");
    }, std::invalid_argument);
}

TEST_F(WalkerSettingsTest, Load_Int_Nonexistent) {
    EXPECT_THROW({
        settings->i("outer_block", "this_isnt_real");
    }, std::logic_error);
}

TEST_F(WalkerSettingsTest, Load_Float) {
    EXPECT_NEAR(settings->f("0.5"), 0.5, 1e-14);
    EXPECT_NEAR(settings->f("outer_block", "0.5"), 0.5, 1e-14);
    EXPECT_NEAR(settings->f("reciprocals", "0.5"), 2.0, 1e-14);
}

TEST_F(WalkerSettingsTest, Load_Float_Empty) {
    EXPECT_THROW({
        settings->f("outer_block", "empty_block");
     }, std::invalid_argument);
}

TEST_F(WalkerSettingsTest, Load_Float_Nonexistent) {
    EXPECT_THROW({
        settings->f("outer_block", "this_isnt_real");
    }, std::logic_error);
}

TEST_F(WalkerSettingsTest, Load_String) {
    EXPECT_STREQ(settings->cstr("outer_block", "text_block"), "Niraj");
}

TEST_F(WalkerSettingsTest, Load_String_Nonexistent) {
    EXPECT_EQ(settings->cstr("outer_block", "this_isnt_real"), nullptr);
}

TEST_F(WalkerSettingsTest, Load_String_As_Int) {
    EXPECT_EQ(settings->i("0.5"), 0);
    EXPECT_THROW({
        settings->i(".5"); // if it doesn't start (after optional whitespace) with a number
    }, std::invalid_argument);
}

TEST_F(WalkerSettingsTest, Load_Bool_Text) {
    EXPECT_TRUE(settings->b("outer_block", "text_block"));
}

TEST_F(WalkerSettingsTest, Load_Bool_False) {
    EXPECT_FALSE(settings->b("false"));
    EXPECT_FALSE(settings->b("False"));
    EXPECT_FALSE(settings->b("zero"));
    EXPECT_FALSE(settings->b("outer_block", "empty_block"));
}

TEST_F(WalkerSettingsTest, Load_Bool_Nonexistent) {
    EXPECT_FALSE(settings->b("outer_block", "this_isnt_real"));
}

TEST_F(WalkerSettingsTest, Exists) {
    EXPECT_TRUE(settings->exists("0.5"));
    EXPECT_TRUE(settings->exists({"outer_block", "empty_block"}));
    EXPECT_FALSE(settings->exists({"outer_block", "this_isnt_real"}));
    EXPECT_FALSE(settings->exists("this_isnt_real"));
}

TEST_F(WalkerSettingsTest, Load_Vector) {
    std::vector<float> v = settings->vf("array");
    EXPECT_EQ(v.size(), 3);
    for (int i = 0; i < 3; i++) {
        EXPECT_EQ(v.at(i), i + 1);
    }
}

TEST_F(WalkerSettingsTest, Colon_Subfield_Access) {
    EXPECT_STREQ(settings->cstr("outer_block.mid_block.inner_block"), "nested");
}

TEST_F(WalkerSettingsTest, Colon_Subfield_Access_Missing) {
    EXPECT_THROW({
        settings->f("outer_block.this_isnt_real");
    }, std::logic_error);
}

TEST_F(WalkerSettingsTest, Subfield_Vector) {
    EXPECT_STREQ(settings->cstr(std::vector<const char *>({"false"})), "false");
    EXPECT_STREQ(settings->cstr({"outer_block", "mid_block", "inner_block"}), "nested");
}

TEST_F(WalkerSettingsTest, Subfield_Vector_Missing) {
    EXPECT_EQ(settings->cstr(std::vector<const char *>({})), nullptr);
    EXPECT_EQ(settings->cstr({"outer_block", "this_isnt_real"}), nullptr);
    EXPECT_EQ(settings->cstr(std::vector<const char *>({"this_isnt_real"})), nullptr);
}

TEST_F(WalkerSettingsTest, Subfield_Vector_Missing_Float) {
    EXPECT_THROW({
        settings->f({"outer_block", "this_isnt_real"});
    }, std::logic_error);
}
