#!/bin/bash
set -e

USAGE="Usage: run_remote_calibration.sh [--pull-only] [--inputs|-i calibration_inputs_json_file] output_tag"
SIMPLEWALKER_ROOT=$(dirname "$(dirname "$(realpath "${BASH_SOURCE[0]}")")")

while [[ $# -gt 0 ]]; do
  case $1 in
    -i|--inputs)
      inputs_file="$2"
      shift && shift
      ;;
    --pull-only)
      PULL_ONLY=true
      shift
      ;;
    --load-pico)
      LOAD_PICO=true
      shift
      ;;
    -*)
      echo "Unknown option $1"
      echo "$USAGE" && exit 2
      ;;
    *)
      if [[ -n $output_tag ]]; then
        echo "Attempted to set output tag twice: $output_tag $1"
        echo "$USAGE" && exit 3
      fi
      output_tag="$1"
      shift
      ;;
  esac
done


pushd "$SIMPLEWALKER_ROOT" > /dev/null

if [[ -z $PULL_ONLY ]]; then  # pull-only mode skips most of this

if [[ -z $output_tag ]]; then
  echo "Missing output tag"
  echo "$USAGE" && exit 1
fi
if [[ -z $inputs_file ]]; then
  inputs_file=settings/motor_calibration_inputs.json
fi
if [[ ! -f ${SIMPLEWALKER_ROOT}/$inputs_file ]]; then
  echo "Inputs file ${SIMPLEWALKER_ROOT}/$inputs_file doesn't exist or is not a valid file"
  exit 4
fi

if [[ -n $LOAD_PICO ]]; then
  # Build and load pico binary; sync motor calibration script and inputs
  cd microcontroller/build && cmake .. > /dev/null && make calibrate_motor && cd ../.. || cd ../..
  # TODO change load_pico.sh to start from any dir, move out of microcontroller dir, accept input file as path
  pushd microcontroller > /dev/null
  scripts/load_pico.sh -r calibrate_motor
  popd > /dev/null
fi

rsync -zaiR calibrate/runMotorCalibration.py $inputs_file pi@raspberrypi:/home/pi/simplewalker/

ssh -l pi raspberrypi "cd simplewalker && python3 calibrate/runMotorCalibration.py $inputs_file $output_tag"


fi  # end if not pull-only mode

echo "Sync motortest files from robot"
rsync -zict pi@raspberrypi:/home/pi/simplewalker/data/m0_"${output_tag}"*.motortest ./data/

uv run --extra calibration calibrate/calibrate_cli.py data/m0_${output_tag}* -m static_fric const_fric s_punch

popd > /dev/null
