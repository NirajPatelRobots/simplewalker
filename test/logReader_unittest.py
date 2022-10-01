""" unittests for python log reader
September 2022
TODO:
"""

import unittest
import os
from logReader import LoadedLog
from typing import List, Dict

TEST_LOG_NAME = "data/logReader_unittest.log"


class TestLoadedLog(unittest.TestCase):
    def tearDown(self):
        if os.path.exists(TEST_LOG_NAME):
            os.remove(TEST_LOG_NAME)

    @staticmethod
    def create_logfile(column_names: Dict[str, int] = None, num_rows: int = 5):
        column_names = column_names or {" accel (3x1)|": 3, " gyro (3x1)| ": 3}
        with open(TEST_LOG_NAME, "wt") as inFile:
            inFile.write(" timestamp  |, ")
            for column_name in column_names:
                inFile.write(column_name + ",")
            inFile.write("\n")
            for i in range(num_rows):
                inFile.write(str(i) + "|, ")
                for column_name in column_names:
                    vector_length = column_names[column_name]
                    if vector_length > 1:
                        inFile.write("(")
                    inFile.write("0.0 " * vector_length)
                    if vector_length > 1:
                        inFile.write(")")
                    inFile.write(",")
                inFile.write("\n")


    def test_create_no_filename(self):
        log = LoadedLog()
        self.assertIsNone(log.loaded_filename)

    def test_create_load(self):
        self.create_logfile()
        log = LoadedLog(TEST_LOG_NAME, [])
        self.assertEqual(TEST_LOG_NAME, log.loaded_filename)

    def test_load(self):
        log = LoadedLog()
        test_length = 10
        self.create_logfile(num_rows=test_length)
        log.load(TEST_LOG_NAME, ["timestamp", "accel", "gyro"])
        self.assertEqual(TEST_LOG_NAME, log.loaded_filename)
        self.assertEqual(test_length, log.timestamp.size)
        self.assertEqual((3, test_length), log.accel.shape)
        self.assertEqual((3, test_length), log.gyro.shape)
        self.assertEqual(0.0, log.accel[2, 0])
        self.assertEqual(2, log.timestamp[2])
        self.assertEqual((1,), log.shapes["timestamp"])
        self.assertEqual((3, 1), log.shapes["accel"])
        self.assertEqual((3, 1), log.shapes["gyro"])

    def test_different_field_names(self):
        log = LoadedLog()
        test_length = 10
        self.create_logfile(column_names={"| sensordata (6)  ": 6, "  quat ( 4x1) ": 4}, num_rows=test_length)
        log.load(TEST_LOG_NAME, ["timestamp", "sensordata", "quat"])
        self.assertEqual(test_length, log.timestamp.size)
        self.assertEqual((6, test_length), log.sensordata.shape)
        self.assertEqual((4, test_length), log.quat.shape)
        self.assertEqual((1,), log.shapes["timestamp"])
        self.assertEqual((6, 1), log.shapes["sensordata"])
        self.assertEqual((4, 1), log.shapes["quat"])

    def test_load_2D_matrix(self):
        log = LoadedLog()
        test_length = 10
        self.create_logfile(column_names={"  matrix ( 3x4) ": 12}, num_rows=test_length)
        log.load(TEST_LOG_NAME, ["timestamp", "matrix"])
        self.assertEqual(test_length, log.timestamp.size)
        self.assertEqual((3, 4, test_length), log.matrix.shape)
        self.assertEqual((1,), log.shapes["timestamp"])
        self.assertEqual((3, 4, 1), log.shapes["matrix"])

    def test_parse_log_fields(self):
        self.create_logfile()
        fields = LoadedLog.parse_log_fields(TEST_LOG_NAME)
        self.assertDictEqual({"timestamp": (1,), "accel": (3, 1), "gyro": (3, 1)}, fields)

    def test_bad_field(self):
        with self.assertRaises(KeyError) as cm:
            self.create_logfile()
            LoadedLog(TEST_LOG_NAME, ["this_doesnt_exist"])
        self.assertIn("this_doesnt_exist", str(cm.exception))
        self.assertIn("accel", str(cm.exception))

    def test_load_bad_file(self):
        with self.assertRaises(FileNotFoundError):
            LoadedLog("this_doesnt_exist", [])


if __name__ == '__main__':
    unittest.main()
