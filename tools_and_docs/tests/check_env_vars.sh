# shellcheck shell=bash

abort() { # Print a message to stderr and abort
  echo "ERROR: $*" >&2
  exit 1
}

check_num() { # Usage: check_num VAR_NAME, accepts decimals and negative values
  local name=$1 value=${!1}
  [[ "$value" =~ ^-?[0-9]*\.?[0-9]+$ ]] || abort "$name='$value', expected a number"
}

check_int() { # Usage: check_int VAR_NAME min max
  local name=$1 value=${!1}
  [[ "$value" =~ ^(0|[1-9][0-9]*)$ ]] || abort "$name='$value', expected an integer (no leading zeros)"
  [ "$value" -ge "$2" ] && [ "$value" -le "$3" ] || abort "$name='$value', expected $2..$3"
}

check_enum() { # Usage: check_enum VAR_NAME allowed1 allowed2 ...
  local name=$1 value=${!1} # ${!1} expands to the value of the variable *named* by $1
  shift # Drop the name, leaving the allowed values in $@
  [[ " $* " == *" $value "* ]] || abort "$name='$value', expected one of: $*" # Padding both sides with spaces forces a whole-word match
}

print_envvars() { # Print the options this run will use
  local file=${BASH_SOURCE[1]} options name # BASH_SOURCE[1] is the script that called us
  options=$(sed -n 's/^\([A-Z_][A-Z_0-9]*\)="\?\${\1:-.*/\1/p' "$file" | tr '\n' ' ') # Option names from its defaults block
  [ -n "$options" ] || abort "print_envvars found no options in $file" # Fail loudly instead of printing nothing
  for name in $options; do echo "$name=${!name}"; done # ${!name} expands to the value of the option named by it
}
