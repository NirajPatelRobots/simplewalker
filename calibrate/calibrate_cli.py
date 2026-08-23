# /// script
# dependencies = ["calibrate"]
# ///
""" TODO:
    Load model but not params from file, maybe plaintext list of model names with nonlin parameter names?
"""
import argparse
from calibrate import examineMotor, printMotorResults, FilterParams
from calibrate_fileIO import load_runs, saveParams, loadParams, saveResultsJson


def parse_args(src=None):
    parser = argparse.ArgumentParser(prog="Calibrate Motor")
    parser.add_argument("filenames", nargs='+', help="Input .motortest files")
    parser.add_argument("--test-only", action="store_true", help="Use parameters from file without optimization.")
    parser.add_argument("-m", "--model", nargs='+',
                        help="Names of model parameters. V and vel are always in the model. Ignored if -i is set.")
    parser.add_argument("-o", "--params-out", help="File to save determined params to")
    parser.add_argument("--results-json")
    parser.add_argument("-i", "--params-in",
                help="Load params from this file. The parameters for testing or the starting point for optimization.")
    filter_group = parser.add_argument_group()
    filter_N_group = filter_group.add_mutually_exclusive_group()
    filter_N_group.add_argument("-T", "--filt-period", type=float)
    filter_N_group.add_argument("-N", "--filt-samples", type=int)
    filter_group.add_argument("--filt-order", type=int)
    filter_group.add_argument("--filt-fcn", choices=FilterParams.fcn_options, default=FilterParams.fcn)
    filter_group.add_argument("--filt-moving-avg", type=int)
    args = parser.parse_args(src)
    if args.test_only and (args.model is not None or args.params_out is not None):
        parser.error("Cannot set model or save params in test-only mode")
    return args


def main():
    args = parse_args()
    testdata = load_runs(args.filenames)
    params = None if args.params_in is None else loadParams(args.params_in)
    filter_params = FilterParams(N=args.filt_samples, T=args.filt_period, order=args.filt_order, fcn=args.filt_fcn,
                                 angle_moving_avg_N=args.filt_moving_avg)
    params, results = examineMotor(testdata, args.model, params, filter_params, args.test_only)
    printMotorResults(params, results)
    if args.params_out is not None:
        saveParams(params, args.params_out)
    if args.results_json is not None:
        saveResultsJson(args.results_json, results, params.lin.keys())


if __name__ == "__main__":
    main()
