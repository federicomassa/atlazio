# To set up a Qt-Ros package (DEPRECATED)

``` 
mkdir -p atlante_ws/src
cd atlante_ws/src
catkin_create_qt_pkg atlazio
cd ..
catkin_make --force-cmake
```

**This is deprecated because it uses old Qt4 style. Instead, just createa a normal ros package and use a simple CMakeLists.txt file with**

```
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
```
(NB Cmake version > 3.0.2)

and remember to put your .ui files inside the same folder of the sources that include ui_<name>.h 

Then, just add
```
find_package(Qt5Widgets REQUIRED)
find_package(catkin REQUIRED COMPONENTS qt_build roscpp)
catkin_package()
target_link_libraries(<exec_name> Qt5::Widgets ${catkin_LIBRARIES})
```

and compile with `catkin_make` as normal.

# To install qcustomplot

* Download from official site (full pkg + shared libs)
* Put sharedlibs folder where qcustomplot.h is
* ``cd sharedlibs-compilation``
* ``qmake;make;``

Now install to root:
```
sudo cp libqcustomplot.so.2.0.0 /usr/lib/
sudo cp libqcustomplotd.so.2.0.0 /usr/lib/
sudo ln -s /usr/lib/libqcustomplot.so.2.0.0 /usr/lib/libqcustomplot.so
sudo ln -s /usr/lib/libqcustomplotd.so.2.0.0 /usr/lib/libqcustomplotd.so
cd ../../
sudo cp qcustomplot.h /usr/include/
cp $ENV{HOME}/atlante_ws/FindQCustomPlot.cmake /usr/share/cmake-3.5/Modules/
```
