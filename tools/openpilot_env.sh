if [ -z "$OPENPILOT_ENV" ]; then
  export PYTHONPATH="$HOME/openpilot"

  unamestr=`uname`

  export OPENPILOT_ENV=1
fi
