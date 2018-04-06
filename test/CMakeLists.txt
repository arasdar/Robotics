#####################################
cmake_minimum_required (VERSION 2.8) 
project (raspicam_simpletest)
find_package(raspicam REQUIRED)
add_executable (raspicam_simpletest raspicam.cpp)  
target_link_libraries (raspicam_simpletest ${raspicam_LIBS})
#####################################

# arasdar@ubuntu-desktop:~/finroc/sources/cpp/projects/raspicam/build$ 
# cmake ../ -Draspicam_DIR=/home/arasdar/indep-projects/export/linux_arm_debug/lib/cmake/ 
# -DCMAKE_C_COMPILER=/usr/bin/arm-linux-gnueabihf-gcc -DCMAKE_CXX_COMPILER=/usr/bin/arm-linux-gnueabihf-g++
