cmake_minimum_required(VERSION 2.8.3)
  
project(reconstructionEvaluator)  

#----------------------- CMAKE MODULES ------------------------ 

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)  
    
# ----------------------- LIBRARIES ---------------------------
find_package(Boost COMPONENTS filesystem system thread REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CGAL REQUIRED)
find_package(MPRF REQUIRED) #multiple precision useful for cgal
find_package(ASSIMP REQUIRED)
find_package(OpenMP)
if (OPENMP_FOUND) 
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
 
  FIND_PACKAGE(X11)
  FIND_PACKAGE(Threads REQUIRED)

  
  link_directories(${X11_LIB_DIRS})
  
  
  include_directories(${X11_INCLUDE_DIR} )
# ----------------------- INCLUDES ----------------------------
include_directories( 
                      ${OpenCV_INCLUDE_DIRS} 
                      ${CGAL_INCLUDE_DIRS}
                      include/reconstructionEvaluator/
                      include/reconstructionEvaluator/camParser/
                      external/glm/glm/
                      external/
                      external/eigen
                      external/assimp/include/assimp
                      ) 
# ----------------------- GCC FLAGS ----------------------------
 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -frounding-math  -lpthread -lX11 -L/usr/X11R6/lib")

# ------------------------ MESSAGES ----------------------------
 
# ------------------------ BUILD -------------------------------

file(GLOB_RECURSE gtComp_SOURCE 
                  src/*.cpp
                  src/camParser/*.cpp
                  test/*.cpp
                  )

add_executable(reconstructionEvaluator ${gtComp_SOURCE})
target_link_libraries(reconstructionEvaluator ${OpenCV_LIBS} 
                                              ${MPRF_LIBRARIES}
                                              ${LAPACK_LIBRARIES}
                                            -lmpfr
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARY} 
                                            ${X11_LIBRARIES} 
                                            )


# ------------------------ INSTALLATION ------------------------


# --------------------------------------------------------------


