#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/justin/admp_ur5e_sim_ws/src/moveit/moveit_ros/visualization"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/justin/admp_ur5e_sim_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/justin/admp_ur5e_sim_ws/install/lib/python3/dist-packages:/home/justin/admp_ur5e_sim_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/justin/admp_ur5e_sim_ws/build" \
    "/usr/bin/python3" \
    "/home/justin/admp_ur5e_sim_ws/src/moveit/moveit_ros/visualization/setup.py" \
    egg_info --egg-base /home/justin/admp_ur5e_sim_ws/build/moveit/moveit_ros/visualization \
    build --build-base "/home/justin/admp_ur5e_sim_ws/build/moveit/moveit_ros/visualization" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/justin/admp_ur5e_sim_ws/install" --install-scripts="/home/justin/admp_ur5e_sim_ws/install/bin"
