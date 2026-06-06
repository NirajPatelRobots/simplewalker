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
    echo "Precision must be an integer, got $PRECISION"
    exit 2
fi

# Convert Bash arrays into JSON arrays for jq (handles empty arrays cleanly)
FIXED_COLS=$(printf '%s\n' "${FIXED_ARR[@]}" | jq -R . | jq -s . 2>/dev/null || echo "[]")
IGNORE_COLS=$(printf '%s\n' "${IGNORE_ARR[@]}" | jq -R . | jq -s . 2>/dev/null || echo "[]")

# Run the dynamic jq transformation
jq -r --argjson fixed "$FIXED_COLS" --argjson ignore "$IGNORE_COLS" --argjson precision "$PRECISION" '
  .tests |
  if length == 0 then "Empty dataset" else
    # Gather all unique keys present across all items
    ([.[] | keys] | flatten | unique) as $all_keys |

    # Drop the keys in the fixed and ignore lists to find the dynamic keys
    ($all_keys - $fixed - $ignore) as $dynamic_keys |

    # Construct final ordered layout (fixed columns first, then dynamic ones)
    ($fixed + $dynamic_keys) as $cols |

    # Generate Header Row
    ($cols | map("| " + .)),

    # Generate Divider Row (---)
    ([range($cols | length) | "---"] | map("| " + .)),
    
    # Generate Data Rows (with $precision significant figures for floats)
    #(.[] as $row | $cols | map("| " + (($row[.] // "-") | tostring)))
    (.[] as $row | $cols | map("| " + (
      $row[.] | 
      if type == "number" then
        if . == 0 then "0"
        else
          # Calculate power of 10 for $precision significant figures
          ((($precision - 1) - (log10 | floor)) | pow(10; .)) as $p |
          ((. * $p | round) / $p | tostring)
        end
      else
        (. // "-") | tostring
      end
    )))

  end | join(" ") + " |"
' "$JSON_FILE"
