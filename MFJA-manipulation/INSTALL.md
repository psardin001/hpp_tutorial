# Install the MFJA manipulation environment

The root README of the
[MFJA 3rd floor repository](https://github.com/psardin001/mfja_3rd_floor_gz)
is the authoritative installation guide. The commands below install the
environment used by this exercise on Ubuntu 24.04.

Configure the ROS 2 Jazzy apt repository using the
[official instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html),
then install the dependencies:

```bash
sudo apt update
sudo apt install -y \
  build-essential doxygen git python3-venv ros-dev-tools \
  ros-jazzy-desktop ros-jazzy-ros-gz \
  ros-jazzy-coal ros-jazzy-control-msgs \
  ros-jazzy-jrl-cmakemodules \
  ros-jazzy-pinocchio ros-jazzy-proxsuite
```

Clone and install the MFJA 3rd floor repository in one work directory:

```bash
export MFJA_WORK_DIR="$HOME/mfja"
mkdir -p "$MFJA_WORK_DIR"
git clone --recurse-submodules \
  https://github.com/psardin001/mfja_3rd_floor_gz.git \
  "$MFJA_WORK_DIR/mfja_3rd_floor_gz"

CMAKE_BUILD_PARALLEL_LEVEL=2 \
  "$MFJA_WORK_DIR/mfja_3rd_floor_gz/install.sh" "$MFJA_WORK_DIR"
```

The installer builds the pinned HPP underlay, the MFJA 3rd floor repository
overlay and the Viser Python environment. Load them through the generated
setup file:

```bash
source "$MFJA_WORK_DIR/setup.bash"
ros2 run mfja_staubli_manipulation_demos room315_check_setup.sh
ros2 run mfja_staubli_manipulation_demos room315_pick_place.sh --build-only
```

Source `$MFJA_WORK_DIR/setup.bash` in each new terminal used for the exercise.
