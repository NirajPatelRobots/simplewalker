import glob
import json
import numpy as np
from os.path import splitext, basename, commonprefix, exists

from motor_model import Params


def saveResultsJson(filename, results, model):
    if exists(filename):
        with open(filename) as inFile:
            output = json.load(inFile)
    else:
        output = {"tests": []}
    this_test = {}
    for k, v in results.items():
        if (type(v) in [np.ndarray, np.float64] and v.size == 1) or type(v) is float:
            this_test[k] = float(v)
        elif (type(v) is np.int64 and v.size == 1) or type(v) is int:
            this_test[k] = int(v)
        elif type(v) is str:
            this_test[k] = v
    this_test["model"] = " ".join(model)
    this_test["input_file_prefix"] = commonprefix(list(results["log_starts"].values())).split("/")[-1]
    for test in output["tests"]:
        if all([k in test and test[k] == v for k, v in this_test.items()]):
            return
    output["tests"].append(this_test)
    with open(filename, "w") as outFile:
        json.dump(output, outFile, indent=2)


def loadRun(filename):
    if "motortest" not in filename:
        filename = filename + ".motortest"
    try:
        with open(filename, 'rb') as file:
            data = dict(np.load(file, allow_pickle=True))
            data["filename"] = filename
            if "t" not in data:
                data["t"] = np.linspace(0., len(data["V"]) * data["dt"], num=len(data["V"]))
            return data
    except Exception as e:
        print("Load failed:", filename, " because \n", e)


def load_runs(filenames):
    testdata = [loadRun(f) for f in sorted([g for f in filenames for g in glob.glob(f)])]  # :P
    names = [splitext(basename(data["filename"]))[0] for data in testdata]
    prefix = commonprefix(names)[:-1] if commonprefix(names).endswith("_a") else commonprefix(names)
    print(f"File prefix: {prefix}, tags: {', '.join([n[len(prefix):] for n in names])}")
    return testdata


def sanitize_dict(d):
    return {k: (sanitize_dict(v) if type(v) is dict
                else v if (v is None or type(v) is int)
                else float(v)) for (k, v) in d.items()}


def saveParams(params, filename = "new"):
    with open(filename if "." in str(filename) else str(filename) + ".json", "w") as file:
        json.dump(obj={"lin": sanitize_dict(params.lin), "nonlin": sanitize_dict(params.nonlin)}, fp=file, indent=2)


def loadParams(filename = "motorparams"):
    with open(filename if "." in str(filename) else str(filename) + ".json") as file:
        try:
            loaded = json.load(file)
        except json.JSONDecodeError:
            print("Could not load params from", filename)
            return Params()
    return Params(sanitize_dict(loaded["lin"]), sanitize_dict(loaded["nonlin"]))
