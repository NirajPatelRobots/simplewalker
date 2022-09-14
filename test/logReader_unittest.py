""" unittests for python log reader
September 2022
TODO:
    create_logfile generic columns
"""

import unittest
import os
from logReader import LoadedLog

TEST_LOG_NAME = "data/logReader_unittest.log"


class TestLoadedLog(unittest.TestCase):
    def tearDown(self):
        if os.path.exists(TEST_LOG_NAME):
            os.remove(TEST_LOG_NAME)

    @staticmethod
    def create_logfile(num_rows: int):
        with open(TEST_LOG_NAME, "wt") as inFile:
            inFile.write(" timestamp  |, accel (3x1)| , gyro (3x1)| \n")
            for i in range(num_rows):
                inFile.write(str(i) + "," + "[0.0 0.0 0.0]," * 2 + "\n")


    def test_create_no_filename(self):
        log = LoadedLog()
        self.assertIsNone(log.loaded_filename)

    def test_create_load(self):
        self.create_logfile(5)
        log = LoadedLog(TEST_LOG_NAME, [], [])
        self.assertEqual(TEST_LOG_NAME, log.loaded_filename)

    def test_load(self):
        log = LoadedLog()
        test_length = 10
        self.create_logfile(test_length)
        log.load(TEST_LOG_NAME, ["timestamp"], ["accel", "gyro"])
        self.assertEqual(TEST_LOG_NAME, log.loaded_filename)
        self.assertEqual(test_length, log.timestamp.size)
        self.assertEqual((3, test_length), log.accel.shape)
        self.assertEqual((3, test_length), log.gyro.shape)
        self.assertEqual(0.0, log.accel[2, 0])
        self.assertEqual(2, log.timestamp[2])

    def test_get_field_names(self):
        log = LoadedLog()
        self.create_logfile(5)
        fields = log.get_field_names(TEST_LOG_NAME)
        self.assertListEqual(["timestamp", "accel (3x1)", "gyro (3x1)"], fields)

    def test_bad_field(self):
        with self.assertRaises(ValueError):
            self.create_logfile(5)
            LoadedLog(TEST_LOG_NAME, ["this_doesnt_exist"], [])

    def test_load_bad_file(self):
        with self.assertRaises(FileNotFoundError):
            LoadedLog("this_doesnt_exist", [], [])


if __name__ == '__main__':
    unittest.main()
