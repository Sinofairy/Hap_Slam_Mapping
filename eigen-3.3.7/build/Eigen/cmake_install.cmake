# Install script for directory: /home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE FILE FILES
    "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/Cholesky"
    "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/CholmodSupport"
    "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/Core"
    "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/Dense"
    "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/Eigen"
    "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/Eigenvalues"
    "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/Geometry"
    "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/Householder"
    "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/IterativeLinearSolvers"
    "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/Jacobi"
    "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/LU"
    "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/MetisSupport"
    "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/OrderingMethods"
    "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/PaStiXSupport"
    "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/PardisoSupport"
    "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/QR"
    "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/QtAlignedMalloc"
    "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/SPQRSupport"
    "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/SVD"
    "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/Sparse"
    "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/SparseCholesky"
    "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/SparseCore"
    "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/SparseLU"
    "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/SparseQR"
    "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/StdDeque"
    "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/StdList"
    "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/StdVector"
    "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/SuperLUSupport"
    "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/UmfPackSupport"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE DIRECTORY FILES "/home/sinofairy/workroot/SlamParse/eigen-3.3.7/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

