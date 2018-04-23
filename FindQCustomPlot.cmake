# - Try to find QCustomPlot
# Once done, this will define
#
#  QCustomPlot_FOUND - system has QCustomPlot
#  QCustomPlot_INCLUDE_DIRS - the QCustomPlot include directories
#  QCustomPlot_LIBRARIES - link these to use QCustomPlot

IF( QCustomPlot_FOUND )
   # in cache already
   SET( QCustomPlot_FIND_QUIETLY TRUE )
ENDIF()

# Include dir
find_path(QCustomPlot_INCLUDE_DIRS
  NAMES qcustomplot.h
  PATHS "/usr/include"
)

# Finally the library itself
find_library(QCustomPlot_LIBRARIES
  NAMES qcustomplot
  PATHS "/usr/lib"
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(QCustomPlot_PROCESS_INCLUDES QCustomPlot_INCLUDE_DIR)
set(QCustomPlot_PROCESS_LIBS QCustomPlot_LIBRARY)

message("-- Found QCustomPlot: ${QCustomPlot_INCLUDE_DIRS} ; ${QCustomPlot_LIBRARIES}")
