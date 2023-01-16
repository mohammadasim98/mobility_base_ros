#! /bin/bash

# Set clock to display seconds
echo "Setting clock to display seconds..."
gsettings set com.canonical.indicator.datetime show-seconds true

# Set power button to shutdown without prompt
echo "Setting power button to shutdown without prompt..."
echo "action=/sbin/poweroff" | sudo tee -a /etc/acpi/events/powerbtn
sudo acpid restart

# Remove unnecessary packages
echo "Removing unnecessary packages..."
sudo apt-get update
sudo apt-get remove -y thunderbird transmission-gtk transmission-common unity-webapps-common brasero-common
sudo apt-get autoremove -y

# Upgrade
echo "Upgrading system..."
sudo apt-get dist-upgrade -y
sudo apt-get update
sudo apt-get dist-upgrade -y
sudo apt-get autoremove -y

# Determine ROS version to install
codename=`lsb_release -sc`
if   [ "$codename" = "trusty" ]; then
  ROS_DISTRO=indigo
elif [ "$codename" = "xenial" ]; then
  ROS_DISTRO=kinetic
else
  echo "Unable to determine ROS version for OS codename '"$codename"'"
  exit 1
fi

# Install ROS
echo "Installing ROS $ROS_DISTRO..."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install -y ros-$ROS_DISTRO-desktop
sudo rosdep init

# Update rosdep rules
echo "Updating rosdep rules..."
rosdep update

# Setup environment
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# Install Mobility Base SDK
echo "Installing Mobility Base SDK..."
#./sdk_install.bash
bash <(wget -q -O - https://bitbucket.org/DataspeedInc/mobility_base_ros/raw/master/mobility_base/scripts/sdk_install.bash)

echo "NUC install: Done"

