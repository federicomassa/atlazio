# To set up a Qt-Ros package

```
mkdir -p atlante_ws/src
cd atlante_ws/src
catkin_create_qt_pkg atlazio
cd ..
catkin_make --force-cmake
```

# To install qcustomplot

* Download from official site (full pkg + shared libs)
* Put sharedlibs folder where qcustomplot.h is
* ``cd sharedlibs-compilation``
* ``qmake;make;``

Now install to root:
```
sudo cp \*.so\* /usr/lib
cd ../../
sudo cp qcustomplot.h /usr/include/
cp $ENV{HOME}/atlante_ws/FindQCustomPlot.cmake /usr/share/cmake-3.5/Modules/
```
