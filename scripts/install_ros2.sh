#!/usr/bin/env bash
# Install ROS 2 Lyrical (ros-core) + foxglove_bridge on Ubuntu 26.10 (arm64/amd64).
#
# There are no ROS packages for 26.10 yet, so this uses the Ubuntu 26.04
# ("resolute") ROS repo. Everything resolves against 26.10 except one library,
# libyaml-cpp0.8, which 26.10 has replaced with a newer soname; we install that
# single .deb from the 26.04 archive (checksum-verified). It coexists with the
# newer libyaml-cpp and nothing else from 26.04 is pulled in.
#
# Run with:  sudo bash ~/misty_ws/scripts/install_ros2.sh
set -euo pipefail

if [[ $EUID -ne 0 ]]; then echo "Run with sudo." >&2; exit 1; fi

ROS_DISTRO=lyrical
ROS_SUITE=resolute
ARCH=$(dpkg --print-architecture)

echo ">>> Adding ROS 2 apt repository (${ROS_SUITE})"
apt-get update
apt-get install -y curl ca-certificates
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.asc
cat > /etc/apt/sources.list.d/ros2.sources <<SRC
Types: deb
URIs: http://packages.ros.org/ros2/ubuntu
Suites: ${ROS_SUITE}
Components: main
Architectures: ${ARCH}
Signed-By: /usr/share/keyrings/ros-archive-keyring.asc
SRC

echo ">>> Installing libyaml-cpp0.8 from Ubuntu 26.04"
if ! dpkg -s libyaml-cpp0.8 >/dev/null 2>&1; then
    case "$ARCH" in
        arm64) MIRROR=http://ports.ubuntu.com/ubuntu-ports
               SHA=a493db8d6b06a6fc24ecee4976c73096397f3856315b24a9b568a829906cb22f ;;
        amd64) MIRROR=http://archive.ubuntu.com/ubuntu; SHA= ;;
        *) echo "Unsupported arch $ARCH" >&2; exit 1 ;;
    esac
    DEB=libyaml-cpp0.8_0.8.0+dfsg-9_${ARCH}.deb
    TMP=$(mktemp -d)
    curl -fsSL "${MIRROR}/pool/main/y/yaml-cpp/${DEB}" -o "${TMP}/${DEB}"
    if [[ -n "$SHA" ]]; then echo "${SHA}  ${TMP}/${DEB}" | sha256sum -c -; fi
    apt-get install -y "${TMP}/${DEB}"
    rm -rf "$TMP"
fi

echo ">>> Installing ROS 2 ${ROS_DISTRO} + foxglove_bridge"
apt-get update
apt-get install -y \
    ros-${ROS_DISTRO}-ros-core \
    ros-${ROS_DISTRO}-foxglove-bridge \
    python3-colcon-common-extensions \
    python3-requests

echo
echo ">>> Done. Next:  source /opt/ros/${ROS_DISTRO}/setup.bash"
