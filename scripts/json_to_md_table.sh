#!/usr/bin/env bash

set -euo pipefail

# Ensure at least the JSON file argument is provided
if [ "$#" -lt 1 ] || [ "$1" == '-h' ]; then
    echo "Usage: $0 <json_file> [col1 col2...] [--ignore <ignored_col1> [ignored_col2...]] [--precision <number>]" >&2
    exit 1
fi

FIXED_ARR=()
IGNORE_ARR=()
PARSING_IGNORED=false
PRECISION=8

JSON_FILE="$1"
shift
while [ "$#" -gt 0 ]; do
    if [ "$1" = --ignore ]; then
        PARSING_IGNORED=true
    elif [ "$1" = --precision ]; then
        PRECISION=$2
        shift
    elif [ "$PARSING_IGNORED" = true ]; then
        IGNORE_ARR+=("$1")
    else
        FIXED_ARR+=("$1")
    fi
    shift
done

if [[ ! "$PRECISION" =~ ^[0-9]+$ ]]; then
    echo "Precision must be an integer, got $PRECISION" >&2 && exit 2
fi

# Convert Bash arrays into JSON arrays for jq (handles empty arrays cleanly)
FIXED_COLS=$(printf '%s\n' "${FIXED_ARR[@]}" | jq -R . | jq -s . 2>/dev/null || echo "[]")
IGNORE_COLS=$(printf '%s\n' "${IGNORE_ARR[@]}" | jq -R . | jq -s . 2>/dev/null || echo "[]")

jq -r --argjson fixed "$FIXED_COLS" --argjson ignore "$IGNORE_COLS" --argjson precision "$PRECISION" '
  def sanitize:  # clean and use $precision significant figures for floats
    if type == "number" then
      if . == 0 then "0"
      else
        # $mult is the 10^n multiplier to shift numbers to $precision digits to be rounded to int
        # Ex: precision=4; value=1.234 -> mult=1000, value=12 -> mult=100, value>1000 -> mult=1
        ([((($precision - 1) - (log10 | floor)) | pow(10; .)), 1] | max) as $mult |
        ((. * $mult | round) / $mult)
      end
    else
      (. // "-")  # Replace null with "-"
    end | tostring
  ;
  def highlight: "<ins>**" + . + "**</ins>" ;
  
  .[] |  # The input to the next section is the array of tests. The outputs are the rows.
  if length == 0 then "Empty dataset" else
    # Gather all unique keys present across all items
    ([.[] | keys] | flatten | unique) as $all_keys |

    # Construct columns: first the fixed list, then the keys in neither list
    ($fixed + ($all_keys - $fixed - $ignore)) as $cols |

    # Generate Header Row
    ($cols | map("| " + .)),

    # Generate Divider Row (---)
    ([range($cols | length) | "---"] | map("| " + .)),
    
    # Save the rows, and save the first row so we can compare other rows to it
    .[] as $rows | .[0] as $compare_row |
    
    # Generate Data Rows
    ($cols | map("| " + (
      ($compare_row[.] | sanitize) as $compare_val |
      $rows[.] | sanitize |
      if . == $compare_val then . else highlight end
    )))

  end | join(" ") + " |"
' "$JSON_FILE"
