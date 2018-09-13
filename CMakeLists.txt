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
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)  
if (OPENMP_FOUND) 
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
 
  FIND_PACKAGE(X11)
  FIND_PACKAGE(Threads REQUIRED)

   
  link_directories(${X11_LIB_DIRS})
  
  
  include_directories(${X11_INCLUDE_DIR} )
find_package(PCL 1.7 REQUIRED) 
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
# ----------------------- INCLUDES ----------------------------
include_directories( 
                      ${GLEW_INCLUDE_DIRS}
                      ${GMPXX_INCLUDE_DIR}
                      ${OpenCV_INCLUDE_DIRS} 
                      ${CGAL_INCLUDE_DIRS}
                      ${SOIL_INCLUDE_DIRS}
                      ${PCL_INCLUDE_DIRS}
                      ${CGAL_INCLUDE_DIRS}
                      ${GLFW_INCLUDE_DIR} 
                      include/reconstructionEvaluator/
                      include/reconstructionEvaluator/camParser/
                      external/glm/glm/
                      external/
                      external/eigen
                      external/assimp/include/assimp
                      ) 
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
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
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${OPENGL_LIBRARIES}
                                            ${GLEW_LIBRARIES}
                                            ${GLFW_INCLUDE_DIRS}
                                            ${MPRF_LIBRARIES}
                                            ${LAPACK_LIBRARIES}
                                            mpfr
                                            glfw
                                            SOIL
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES}
                                            ${ASSIMP_LIBRARY} 
                                            ${X11_LIBRARIES} 
                                            ${PCL_LIBRARIES}
                                            )


# ------------------------ INSTALLATION ------------------------


# --------------------------------------------------------------


