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

message(BULLET_INCLUDE_DIRS: ${Bullet_INCLUDE_DIRS})

set(Bullet_LIBRARIES ${DRAKE_DIR}/bullet/pod-build/src/BulletCollision/libBulletCollision.so ${DRAKE_DIR}/bullet/pod-build/src/LinearMath/libLinearMath.so)


set(SNOPT_INCLUDE_DIRS ${DRAKE_DIR}/snopt/pod-build/include/snopt/ /home/drc/Software/drake-distro/snopt/libf2c/)

set(SNOPT_LIBRARIES ${DRAKE_DIR}/snopt/pod-build/lib/libf2c.a ${DRAKE_DIR}/snopt/pod-build/lib/libsnblas.a ${DRAKE_DIR}/snopt/pod-build/lib/libsnopt_c.a ${DRAKE_DIR}/snopt/pod-build/lib/libsnprint.a ${DRAKE_DIR}/snopt/pod-build/lib/libliblast.a ${DRAKE_DIR}/snopt/pod-build/lib/libsnopt.a)

set(GFORTRAN_LIBRARIES /usr/lib/x86_64-linux-gnu/libgfortran.so.3)

set(F2C_LIBRARIES ${DRAKE_DIR}/snopt/pod-build/lib/libf2c.a)



include_directories(${SNOPT_INCLUDE_DIRS} ${Bullet_INCLUDE_DIRS})


add_mex(replanFunnels_mex replanFunnels_mex.cpp)
target_link_libraries (replanFunnels_mex ${Bullet_LIBRARIES})
target_link_libraries (replanFunnels_mex ${SNOPT_LIBRARIES})
target_link_libraries (replanFunnels_mex ${GFORTRAN_LIBRARIES})
target_link_libraries (replanFunnels_mex ${F2C_LIBRARIES})
target_link_libraries(replanFunnels_mex -lrt)


add_mex(isCollisionFree_mex isCollisionFree_mex.cpp)
target_link_libraries (isCollisionFree_mex ${Bullet_LIBRARIES})
target_link_libraries(isCollisionFree_mex -lrt)

add_mex(ptToPolyBullet_mex ./bullet_collision/ptToPolyBullet_mex.cpp)
target_link_libraries (ptToPolyBullet_mex ${Bullet_LIBRARIES})
target_link_libraries(ptToPolyBullet_mex -lrt)






