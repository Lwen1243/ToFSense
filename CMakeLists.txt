cmake_minimum_required(VERSION 3.0.0)
project(Tof-distance VERSION 0.1.0)

include(CTest)
enable_testing()

add_executable(
<<<<<<< HEAD
<<<<<<< HEAD
=======
  nlink_unpack
>>>>>>> update
=======
>>>>>>> 9b705162ebba7b40ae702c428dcd065629969d93
  nlink_unpack/nlink_utils.c
  nlink_unpack/nlink_linktrack_nodeframe0.c
  nlink_unpack/nlink_linktrack_nodeframe1.c
  nlink_unpack/nlink_linktrack_nodeframe2.c
  nlink_unpack/nlink_linktrack_nodeframe3.c
  nlink_unpack/nlink_linktrack_aoa_nodeframe0.c
  nlink_unpack/nlink_linktrack_tagframe0.c
  nlink_unpack/nlink_linktrack_anchorframe0.c
  nlink_unpack/nlink_tofsense_frame0.c
  nlink_unpack/nlink_tofsensem_frame0.c
  get_data.c
<<<<<<< HEAD
<<<<<<< HEAD
  main.c
=======
  main.h
>>>>>>> update
=======
  main.c
>>>>>>> 9b705162ebba7b40ae702c428dcd065629969d93
  )

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
