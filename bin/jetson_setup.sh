#!/usr/bin/env bash

# Setup script for Nvidia Jetson Nano boards.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# September 2, 2023

set -o errexit
set -o nounset
set -o pipefail
if [[ "${TRACE-0}" == "1" ]]; then set -o xtrace; fi

# Verify that the PWD is the project root directory
CURR_DIR=${PWD##*/}
INIT_DIR=$PWD
REQ_CURR_DIR="traxxas1"
if [[ $CURR_DIR != "$REQ_CURR_DIR" ]]; then
  echo >&2 "ERROR: Wrong path, this script must run inside $REQ_CURR_DIR"
  return 1
fi

# Disable serial console
echo "Disabling Nvidia serial console..."
sleep 1
sudo systemctl stop nvgetty
sudo systemctl disable nvgetty
udevadm trigger

sudo apt-get update

# Remove some unnecessary packages
echo "Removing unnecessary packages..."
sleep 1
sudo apt-get purge -y \
  libreoffice* \
  modemmanager

# Install basic utilities
echo "Installing basic utilities..."
sleep 1
sudo apt-get install -y --no-install-recommends \
  apt-utils \
  automake \
  build-essential \
  cmake \
  curl \
  gcc \
  gdb \
  gedit \
  gparted \
  gtk3-nocsd \
  htop \
  lm-sensors \
  lsb-release \
  make \
  minicom \
  nano \
  neofetch \
  openjdk-11-jdk \
  pigz \
  python3 \
  python3-pip \
  rsync \
  rt-tests \
  screen \
  ssh \
  sshfs \
  valgrind \
  vim \
  wget
sudo -H pip3 install -U setuptools
sudo -H pip3 install -U jetson-stats

# Update system and clean up
echo "Updating current packages and cleaning up..."
sleep 1
sudo apt-get upgrade -y
sudo apt-get autoclean
sudo apt-get autoremove -y

# Install Docker Compose
echo "Installing Docker Compose v2..."
sleep 1
sudo mkdir -p /usr/local/lib/docker/cli-plugins
sudo curl -SL https://github.com/docker/compose/releases/latest/download/docker-compose-linux-aarch64 -o /usr/local/lib/docker/cli-plugins/docker-compose
sudo chmod +x /usr/local/lib/docker/cli-plugins/docker-compose
sudo ln -s /usr/local/lib/docker/cli-plugins/docker-compose /usr/local/bin/docker-compose

# Create docker group and add user to it
echo "Creating new group for Docker users and adding $USER to it..."
sleep 1
sudo groupadd docker || true
sudo usermod -aG docker "$USER" || true

# Add current user to hardware access groups
HW_GROUPS=(
  "adm"
  "dialout"
  "plugdev"
  "tty"
  "uucp"
  "video")

echo "Adding current user $USER to hardware access groups..."
sleep 1
for GROUP in "${HW_GROUPS[@]}"; do
  if ! groups | grep -q "$GROUP"; then
    sudo usermod -a -G "$GROUP" "$USER" || true
    echo "$USER added to $GROUP"
  fi
done

cd "$INIT_DIR" || return 1
echo "Done! Next run 'bin/container_setup.sh' to setup the Docker container on this board."
echo "The system will now be restarted to apply all changes..."
sleep 5
sudo reboot