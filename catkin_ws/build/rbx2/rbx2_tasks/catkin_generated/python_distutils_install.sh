#!/bin/sh -x

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

cd "/home/yzheng/catkin_ws/src/rbx2/rbx2_tasks"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
/usr/bin/env \
    PYTHONPATH="/home/yzheng/catkin_ws/install/lib/python2.7/dist-packages:/home/yzheng/catkin_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/yzheng/catkin_ws/build" \
    "/usr/bin/python" \
    "/home/yzheng/catkin_ws/src/rbx2/rbx2_tasks/setup.py" \
    build --build-base "/home/yzheng/catkin_ws/build/rbx2/rbx2_tasks" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/yzheng/catkin_ws/install" --install-scripts="/home/yzheng/catkin_ws/install/bin"
