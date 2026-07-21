# Sourced by the run/build scripts to fail fast on invalid environment variables

die() { echo "ERROR: $*" >&2; exit 1; } # Message to stderr, abort

check_names() { # Abort on an environment variable that looks like a misspelled option, takes no arguments
  local file=${BASH_SOURCE[1]} options prefixes name upper # BASH_SOURCE[1] is the script that called us
  options=$(sed -n 's/^\([A-Z_][A-Z_0-9]*\)="\?\${\1:-.*/\1/p' "$file" | tr '\n' ' ') # Option names from its defaults block
  [ -n "$options" ] || die "check_names found no options in $file" # Fail loudly instead of checking nothing
  prefixes=$(tr ' ' '\n' <<< "$options" | cut -c1-3 | tr '\n' ' ') # First three letters of each option
  for name in $(compgen -e); do # Every variable in the environment, ambient ones included
    [[ " $options " == *" $name "* ]] && continue # A correctly spelled option
    upper=${name^^} # Separate expansion, so lowercase 'headless' is caught too
    [[ " $prefixes " == *" ${upper:0:3} "* ]] && die "'$name' is not an option of ${file##*/}, expected one of: $options"
  done
  return 0 # The loop ends on a failed test, return explicitly for set -e
}

check_num() { # Usage: check_num VAR_NAME, accepts decimals and negative values
  local name=$1 value=${!1}
  [[ "$value" =~ ^-?[0-9]*\.?[0-9]+$ ]] || die "$name='$value', expected a number"
}

check_int() { # Usage: check_int VAR_NAME min max
  local name=$1 value=${!1}
  [[ "$value" =~ ^[0-9]+$ ]] || die "$name='$value', expected an integer"
  [ "$value" -ge "$2" ] && [ "$value" -le "$3" ] || die "$name='$value', expected $2..$3"
}

check_enum() { # Usage: check_enum VAR_NAME allowed1 allowed2 ...
  local name=$1 value=${!1} # ${!1} expands to the value of the variable *named* by $1
  shift # Drop the name, leaving the allowed values in $@
  [[ " $* " == *" $value "* ]] || die "$name='$value', expected one of: $*" # Padding both sides with spaces forces a whole-word match
}