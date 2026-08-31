# Install the MFJA manipulation environment

The root README of the
[MFJA 3rd floor repository](https://github.com/psardin001/mfja_3rd_floor_gz)
is the authoritative installation guide. The commands below install the
environment used by this exercise on Ubuntu 24.04.

Configure the ROS 2 Jazzy apt repository using the
[official instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html),
then add the Robotpkg Noble repository:

```bash
sudo apt update
sudo apt install -y curl
sudo install -d -m 0755 /etc/apt/keyrings
curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
  | sudo tee /etc/apt/keyrings/robotpkg.asc >/dev/null
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub noble robotpkg" \
  | sudo tee /etc/apt/sources.list.d/robotpkg.list >/dev/null
```

Install ROS 2, the build tools, HPP 9.0.2, and the HPP viewer:

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake doxygen git python3-venv ros-dev-tools \
  ros-jazzy-desktop ros-jazzy-ros-gz \
  ros-jazzy-control-msgs \
  robotpkg-py312-hpp-python=9.0.2 \
  robotpkg-py312-qt5-hpp-gepetto-viewer=9.0.2
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

Robotpkg supplies the main HPP stack and viewer. The installer builds pinned
TOPPRA, `hpp-toppra`, and `hpp-exec` additions, the MFJA 3rd floor overlay, and
the Viser Python environment. Load them through the generated setup file:

```bash
source "$MFJA_WORK_DIR/setup.bash"
ros2 run mfja_staubli_manipulation_demos room315_check_setup.sh
ros2 run mfja_staubli_manipulation_demos room315_pick_place.sh --build-only
```

Source `$MFJA_WORK_DIR/setup.bash` in each new terminal used for the exercise.
