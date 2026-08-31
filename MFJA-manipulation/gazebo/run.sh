#!/usr/bin/env bash
# Run the optional Gazebo demonstration in an isolated ROS domain.
set -euo pipefail

GAZEBO_DIR=$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd -P)
TUTORIAL_DIR=$(cd -- "$GAZEBO_DIR/.." && pwd -P)
MFJA_WORK_DIR=${MFJA_WORK_DIR:-$HOME/mfja}
MFJA_SETUP=${MFJA_SETUP:-$MFJA_WORK_DIR/setup.bash}

case ${1:-} in
  launch|execute)
    gazebo_command=$1
    ;;
  *)
    echo "Usage: $0 {launch|execute}" >&2
    exit 2
    ;;
esac

if [[ ! -f "$MFJA_SETUP" ]]; then
  echo "MFJA 3rd floor repository setup not found at '$MFJA_SETUP'." >&2
  echo "Follow MFJA-manipulation/INSTALL.md or set MFJA_SETUP." >&2
  exit 1
fi

set +u
unset PYTHONPATH
# shellcheck disable=SC1090
source "$MFJA_SETUP"
set -u

export ROS_DOMAIN_ID=${GAZEBO_ROS_DOMAIN_ID:-42}
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
cd "$TUTORIAL_DIR"

if [[ $gazebo_command == launch ]]; then
  exec python3 -m gazebo.launch_gazebo
fi
exec python3 -m gazebo.execute
