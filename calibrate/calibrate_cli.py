""" TODO:
    Separate sub-commands for train and test (and fourier analysis?)
    Load model but not params from file, maybe plaintext list of model names?
    dump results to file? to display in tables?
"""
import argparse
import glob
from os.path import basename, splitext, commonprefix
from calibrate import loadRun, examineMotor, printMotorResults, FilterParams
from motor_model import saveParams, loadParams


def parse_args(src=None):
    parser = argparse.ArgumentParser(prog="Calibrate Motor")
    parser.add_argument("filenames", nargs='+', help="Input .motortest files")
    parser.add_argument("-m", "--model", nargs='+',
                        help="Names of model parameters. V and vel are always in the model. Ignored if -i is set.")
    io_group = parser.add_mutually_exclusive_group()
    io_group.add_argument("-i", "--params-in",
                          help="Skip parameter determination and load params from this file instead")
    io_group.add_argument("-o", "--params-out", help="File to save determined params to")
    filter_group = parser.add_argument_group()
    filter_N_group = filter_group.add_mutually_exclusive_group()
    filter_N_group.add_argument("-T", "--filt-period", type=float)
    filter_N_group.add_argument("-N", "--filt-samples", type=int)
    filter_group.add_argument("--filt-order", default=6, type=int)
    filter_group.add_argument("--filt-fcn", choices=['butter', 'cheby'], default="butter")
    return parser.parse_args(src)


def load_runs(filenames):
    testdata = [loadRun(f) for f in sorted([g for f in filenames for g in glob.glob(f)])]  # :P
    names = [splitext(basename(data["filename"]))[0] for data in testdata]
    prefix = commonprefix(names)[:-1] if commonprefix(names).endswith("_a") else commonprefix(names)
    print(f"File prefix: {prefix}, tags: {', '.join([n[len(prefix):] for n in names])}")
    return testdata


def main():
    args = parse_args()
    testdata = load_runs(args.filenames)
    params = None if args.params_in is None else loadParams(args.params_in)
    filter_params = FilterParams(N=args.filt_samples, T=args.filt_period, order=args.filt_order, fcn=args.filt_fcn)
    params, results = examineMotor(testdata, args.model, params, filter_params)
    printMotorResults(params, results)
    if args.params_out is not None:
        saveParams(params, args.params_out)


if __name__ == "__main__":
    main()
