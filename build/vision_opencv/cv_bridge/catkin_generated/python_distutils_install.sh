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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/cv_bridge"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/avinash/Deep_Learning_Crazyflie/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/avinash/Deep_Learning_Crazyflie/install/lib/python2.7/dist-packages:/home/avinash/Deep_Learning_Crazyflie/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/avinash/Deep_Learning_Crazyflie/build" \
    "/usr/bin/python" \
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/cv_bridge/setup.py" \
    build --build-base "/home/avinash/Deep_Learning_Crazyflie/build/vision_opencv/cv_bridge" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/avinash/Deep_Learning_Crazyflie/install" --install-scripts="/home/avinash/Deep_Learning_Crazyflie/install/bin"
