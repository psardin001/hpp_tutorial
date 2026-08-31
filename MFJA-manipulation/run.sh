#!/usr/bin/env bash
# Run a command in the MFJA 3rd floor repository environment.
set -euo pipefail

TUTORIAL_DIR=$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd -P)
MFJA_WORK_DIR=${MFJA_WORK_DIR:-$HOME/mfja}
MFJA_SETUP=${MFJA_SETUP:-$MFJA_WORK_DIR/setup.bash}

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

TUTORIAL_PYTHON=${TUTORIAL_PYTHON:-python3}
if ! command -v "$TUTORIAL_PYTHON" >/dev/null 2>&1; then
  echo "Tutorial Python '$TUTORIAL_PYTHON' is unavailable." >&2
  exit 1
fi
if ! "$TUTORIAL_PYTHON" -c \
  'import sys; sys.exit(sys.version_info[:2] != (3, 12))'; then
  echo "The MFJA manipulation environment requires Python 3.12." >&2
  exit 1
fi
if ! "$TUTORIAL_PYTHON" - <<'PY'
import sys
from importlib.metadata import version

import coal
import pinocchio

sys.modules["hppfcl"] = coal
import hpp_exec
import pyhpp
import pyhpp_viser
import rclpy
import trimesh
import viser

assert version("viser") == "1.0.30"
PY
then
  echo "The HPP/ROS/Viser Python environment is incomplete." >&2
  echo "Follow MFJA-manipulation/INSTALL.md." >&2
  exit 1
fi

if [[ $# -eq 0 ]]; then
  set -- "$TUTORIAL_PYTHON" -i exercise.py
elif [[ $1 == python3 ]]; then
  shift
  set -- "$TUTORIAL_PYTHON" "$@"
fi

cd "$TUTORIAL_DIR"
exec "$@"
