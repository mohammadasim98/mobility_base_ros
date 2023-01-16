# Mobility Base
Software supporting the Mobility Base robot from Dataspeed Inc.

# Overview
|                  |                                                             |
|------------------|-------------------------------------------------------------|
|**Documentation** | http://mbsdk.dataspeedinc.com                               |
|**Issues**        | https://bitbucket.org/dataspeedinc/mobility_base_ros/issues |

## Install/Update the Mobility Base SDK (binary)
```bash
bash <(wget -q -O - https://bitbucket.org/DataspeedInc/mobility_base_ros/raw/master/mobility_base/scripts/sdk_install.bash)
```

## Install the NUC workstation from scratch including the Mobility Base SDK
```bash
bash <(wget -q -O - https://bitbucket.org/DataspeedInc/mobility_base_ros/raw/master/mobility_base/scripts/nuc_install.bash)
```

## Checkout source
Note: This will not include the mobility_base_driver package which can only be found in the binary downloads.  
Note: This must be run from the root of a catkin workspace, or an empty folder you wish to become a cakin workspace.
```bash
wstool init src
wget https://bitbucket.org/DataspeedInc/mobility_base_ros/raw/master/mobility_base.rosinstall -O /tmp/mobility_base.rosinstall
wstool merge -t src /tmp/mobility_base.rosinstall
wstool update -t src
rosdep update && rosdep install --from-paths src --ignore-src -r
```

## Included DataspeedInc repos
|          Name           |                            Link                            |
|-------------------------|------------------------------------------------------------|
| mobility_base_ros       | https://bitbucket.org/DataspeedInc/mobility_base_ros       |

## Included 3rd party repos
|          Name           |                            Link                            |
|-------------------------|------------------------------------------------------------|
| usb_cam                 | https://github.com/bosch-ros-pkg/usb_cam.git               |
| velodyne                | https://github.com/ros-drivers/velodyne.git                |

## Additional repos (not included)
|          Name           |                            Link                            |
|-------------------------|------------------------------------------------------------|
| mobility_base_simulator | https://bitbucket.org/DataspeedInc/mobility_base_simulator |
