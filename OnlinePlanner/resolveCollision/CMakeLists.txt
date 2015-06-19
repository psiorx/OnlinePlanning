cmake_minimum_required (VERSION 2.8.6)

ADD_DEFINITIONS(
    -std=c++0x # Or -std=c++0x
    -largeArrayDims
    # Other flags
)

include(${OnlinePlanning_SOURCE_DIR}/cmake/mex.cmake)
mex_setup()
# find_package (Bullet REQUIRED)
# find_package doesn't seem to work for bullet, so I'm doing this manually for now. 

set(DRAKE_DIR /home/drc/drake-distro)

set(Bullet_INCLUDE_DIRS ${DRAKE_DIR}/bullet/bullet-2.81-rev2613/src/)

set(Bullet_LIBRARIES ${DRAKE_DIR}/bullet/pod-build/src/BulletCollision/libBulletCollision.so ${DRAKE_DIR}/bullet/pod-build/src/LinearMath/libLinearMath.so)

include_directories(${Bullet_INCLUDE_DIRS})


add_mex(testResolveCollision testResolveCollision.cpp)
target_link_libraries (testResolveCollision ${Bullet_LIBRARIES})
target_link_libraries(testResolveCollision -lrt)


add_mex(shiftFunnel_qcqp_forces_mex shiftFunnel_qcqp_forces_mex.cpp)
target_link_libraries (shiftFunnel_qcqp_forces_mex ${Bullet_LIBRARIES})
target_link_libraries(shiftFunnel_qcqp_forces_mex -lrt)

