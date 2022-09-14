""" Read saved logs
September 2022
TODO:
    auto detect array size from log
    way to return one at a time (generator?)
"""

import numpy as np
import csv
from typing import List


class LoadedLog:
    def __init__(self, filename: str = None,
                 scalar_fieldnames: List[str] = None, vector_fieldnames: List[str] = None, loud: bool = False):
        self.loaded_filename = None
        self.loud = loud
        if filename is not None:
            self.load(filename, scalar_fieldnames, vector_fieldnames)
        else:
            self.reset_data([], [])

    @staticmethod
    def logged_str_to_array(text: str) -> np.array:
        return np.array(text.strip(" []").split(), dtype=float).reshape((3, 1))

    @staticmethod
    def get_field_names(filename: str) -> List[str]:
        with open(filename, 'r') as inFile:
            reader = csv.reader(inFile)
            fields = next(reader)
        return [f.strip(" |") for f in fields]

    def load(self, filename: str, scalar_fieldnames: List[str], vector_fieldnames: List[str]) -> None:
        self.reset_data(scalar_fieldnames, vector_fieldnames)
        log_fields = self.get_field_names(filename)
        if self.loud:
            print("Opened log", filename, "fields:", log_fields)
        with open(filename, 'r') as inFile:
            reader = csv.reader(inFile)
            next(reader)
            for row in reader:
                for name in scalar_fieldnames:
                    self.fields[name] = np.append(self.fields[name], float(row[log_fields.index(name)]))
                for name in vector_fieldnames:
                    self.fields[name] = np.hstack((self.fields[name],
                                                   self.logged_str_to_array(row[log_fields.index(name + ' (3x1)')])))
        for name in self.fields:
            setattr(self, name, self.fields[name])
        self.loaded_filename = filename

    def reset_data(self, scalar_fieldnames: List[str], vector_fieldnames: List[str]):
        self.fields = {}
        for name in scalar_fieldnames:
            self.fields[name] = np.empty(0)
        for name in vector_fieldnames:
            self.fields[name] = np.empty((3, 0))
