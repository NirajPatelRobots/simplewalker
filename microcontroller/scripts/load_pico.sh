#!/bin/bash

# parse args
local_build_dir=build
for arg in "$@"; do
  if [ "$arg" == "-r" ] || [ "$arg" == "--remote" ]; then
    REMOTE=true
  elif [[ $arg =~ -* ]]; then
    uf2_file=$arg
    if [ -f "$uf2_file" ]; then
      local_build_dir="$(dirname "$uf2_file")"
    fi
  else
    echo unrecognized arg: "$arg"
  fi
done
if [ ! -d "$local_build_dir" ]; then
  local_build_dir="$(dirname "$(dirname $(realpath "${BASH_SOURCE[0]}"))")/$local_build_dir"
fi

# load .uf2 files to remote
if [ $REMOTE ]; then
  build_dir="/home/pi/simplewalker/microcontroller/build"
  remote_cmd="ssh -l pi raspberrypi "

  sync_info="$(rsync -zaic --timeout=5 $local_build_dir/*.uf2 pi@raspberrypi:$build_dir/ 2> errFile)"
  if [ -s errFile ]; then
    echo "$sync_info"
    cat errFile
    rm errFile
    exit 1
  fi
  rm errFile
  if [ -n "$sync_info" ]; then
    echo "$sync_info"
  else
    echo "pico binaries are up to date."
  fi
else
  build_dir=$local_build_dir
  remote_cmd=""
fi

# find picotool and get picotool info
if [ -x "$(${remote_cmd}command -v picotool)" ] ||  [ "$(${remote_cmd}which picotool)" ]; then
  pico_cmd="${remote_cmd}sudo picotool"
elif [ "${PICOTOOL_FETCH_FROM_GIT_PATH}" ]; then
  pico_cmd="${remote_cmd}sudo $PICOTOOL_FETCH_FROM_GIT_PATH/picotool/picotool"
else
  echo "Error: can't find picotool"
  exit 1
fi
picotool_info=$(${pico_cmd} info -blp)

# if no file arg, list available uf2 files and picotool info then exit
if [ -z "$uf2_file" ]; then
  ${remote_cmd}ls -gGh $build_dir/*.uf2  2> errFile
    if [ -s errFile ]; then
      if grep -q "No such file or directory" errFile; then
        echo "No built .uf2 files in build directory $build_dir"
      else
        cat errFile
      fi
    fi
    rm errFile
  echo "$picotool_info"
  exit 0
fi

# check uf2 file
if [[ ! $uf2_file =~ \.uf2 ]]; then
  uf2_file="$uf2_file.uf2"
fi
if [[ ! "$(${remote_cmd}ls "$build_dir")" =~ $uf2_file ]]; then
  echo "$uf2_file does not exist"
  exit 1
fi

# reboot pico if necessary
if [[ ! $picotool_info =~ 'Program Information' ]]; then
  echo "rebooting pico into BOOTSEL mode..."
  pico_reboot_reply=$(${pico_cmd} reboot --usb -f)
  if [[ $pico_reboot_reply =~ 'No accessible' ]]; then
    echo "$pico_reboot_reply"
    exit 1
  fi
fi

# load
${pico_cmd} load -v -u "${build_dir}/${uf2_file}"
${pico_cmd} info -blp
${pico_cmd} reboot
