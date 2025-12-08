# generated from colcon_core/shell/template/package.sh.em

# This script extends the environment for this package.

# function to prepend a value to a variable
# which uses colons as separators
# duplicates as well as trailing separators are avoided
# first argument: the name of the result variable
# second argument: the value to be prepended
_colcon_prepend_unique_value() {
  # arguments
  _listname="$1"
  _value="$2"

  # get values from variable
  eval _values=\"\$$_listname\"
  # backup the field separator
  _colcon_prepend_unique_value_IFS=$IFS
  IFS=":"
  # start with the new value
  _all_values="$_value"
  # workaround SH_WORD_SPLIT not being set in zsh
  if [ "$(command -v colcon_zsh_convert_to_array)" ]; then
    colcon_zsh_convert_to_array _values
  fi
  # iterate over existing values in the variable
  for _item in $_values; do
    # ignore empty strings
    if [ -z "$_item" ]; then
      continue
    fi
    # ignore duplicates of _value
    if [ "$_item" = "$_value" ]; then
      continue
    fi
    # keep non-duplicate values
    _all_values="$_all_values:$_item"
  done
  unset _item
  # restore the field separator
  IFS=$_colcon_prepend_unique_value_IFS
  unset _colcon_prepend_unique_value_IFS
  # export the updated variable
  eval export $_listname=\"$_all_values\"
  unset _all_values
  unset _values

  unset _value
  unset _listname
}

# do not unset _colcon_prepend_unique_value since it might be used by non-primary shell hooks
