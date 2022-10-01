""" Read saved logs
September 2022
TODO:
    way to return one at a time (generator?)
"""

import numpy as np
import csv
from typing import List, Dict, Tuple
import re
from collections import OrderedDict


class LoadedLog:
    def __init__(self, filename: str = None, field_names: List[str] = None, loud: bool = False):
        self.loaded_filename = None
        self.loud = loud
        if filename is not None:
            self.load(filename, field_names)
        else:
            self.reset_data([])

    @staticmethod
    def logged_str_to_array(text: str, shape: Tuple[int]) -> np.array:
        return np.array(text.replace('[', '').replace(']', '').split(), dtype=float).reshape(shape)

    @staticmethod
    def parse_log_fields(filename: str) -> Dict[str, Tuple[int]]:
        with open(filename, 'r') as inFile:
            reader = csv.reader(inFile)
            fieldnames = [f.strip(" |") for f in next(reader) if len(f) > 0]
        fields = OrderedDict()
        for name in fieldnames:
            match = re.match(r"(.+)\((.+)\)", name)
            if match is None:
                fields[name] = (1,)
            else:
                shape = tuple([int(d) for d in match.group(2).split("x")])
                if not shape[-1] == 1:
                    shape += (1,)
                fields[match.group(1).strip()] = shape
        return fields

    def load(self, filename: str, field_names: List[str]) -> None:
        self.reset_data(field_names)
        log_fields = self.parse_log_fields(filename)
        if self.loud:
            print("Opened log", filename, "fields:", log_fields)
        for name in field_names:
            try:
                self.fields[name] = np.empty(shape=log_fields[name][:-1] + (0,))
            except KeyError:
                raise KeyError(filename + " field '" + name + "' not in log fields " + str(list(log_fields.keys())))
        field_idxs = {field_name: list(log_fields.keys()).index(field_name) for field_name in field_names}
        with open(filename, 'r') as inFile:
            reader = csv.reader(inFile)
            next(reader)
            for row in reader:
                for name in field_names:
                    shape = log_fields[name]
                    field_text = row[field_idxs[name]].replace('|', '')
                    if shape == (1,):
                        self.fields[name] = np.append(self.fields[name], float(field_text))
                    else:
                        self.fields[name] = np.concatenate((self.fields[name],
                                                            self.logged_str_to_array(field_text, shape)),
                                                           axis=-1)
        for name in self.fields:
            setattr(self, name, self.fields[name])
        self.loaded_filename = filename
        self.shapes = log_fields

    def reset_data(self, field_names: List[str]):
        self.fields = {}
        self.shapes = {}
        for name in field_names:
            self.fields[name] = np.empty(0)
