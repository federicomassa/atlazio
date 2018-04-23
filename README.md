--- To set up a Qt-Ros package ---

mkdir -p atlante_ws/src
cd atlante_ws/src
catkin_create_qt_pkg atlazio
cd ..
catkin_make --force-cmake

----- To install qcustomplot ------

1. Download from official site (full pkg + shared libs)
2. Put sharedlibs folder where qcustomplot.h is
3. cd sharedlibs-compilation
4. qmake;make;

Now install to root:
5. sudo cp *.so* /usr/lib
6. cd ../../
7. sudo cp qcustomplot.h /usr/include/

