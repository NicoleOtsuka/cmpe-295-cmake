cmake_minimum_required(VERSION 2.8.9)
project(libipif)
set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Release)
set(IPIF_PUBLIC_HDRS
	include/ipif.h
	include/utilis.h
)

set(INSTALL_SRC_DIR "${CMAKE_INSTALL_PREFIX}/src" CACHE PATH "Installation directory for source files")
set(INSTALL_BIN_DIR "${CMAKE_INSTALL_PREFIX}/bin" CACHE PATH "Installation directory for executables")
set(INSTALL_LIB_DIR "${CMAKE_INSTALL_PREFIX}/lib" CACHE PATH "Installation directory for libraries")
set(INSTALL_INC_DIR "${CMAKE_INSTALL_PREFIX}/include" CACHE PATH "Installation directory for headers")
set(INSTALL_MAN_DIR "${CMAKE_INSTALL_PREFIX}/share/man" CACHE PATH "Installation directory for manual pages")

include_directories(include)

file(GLOB SOURCES_LIB "src/*.c")
file(GLOB SOURCES_LOOP "test/test_loop.c" "src/*.c")
file(GLOB SOURCES_ZERO "test/test_zero.c" "src/*.c")

add_library(ipif SHARED ${SOURCES_LIB})
add_executable(test_loop ${SOURCES_LOOP})
add_executable(test_zero ${SOURCES_ZERO})
find_package(Threads REQUIRED)
if(THREADS_HAVE_PTHREAD_ARG)
	target_compile_options(PUBLIC test_loop "-pthread")
	target_compile_options(PUBLIC test_zero "-pthread")
endif()
if(CMAKE_THREAD_LIBS_INIT)
	target_link_libraries(test_loop "${CMAKE_THREAD_LIBS_INIT}")
	target_link_libraries(test_zero "${CMAKE_THREAD_LIBS_INIT}")
endif()

install(TARGETS ipif DESTINATION "${INSTALL_LIB_DIR}" CONFIGURATIONS Debug)
install(TARGETS test_loop DESTINATION "${INSTALL_BIN_DIR}" CONFIGURATIONS Debug)
install(TARGETS test_zero DESTINATION "${INSTALL_BIN_DIR}" CONFIGURATIONS Debug)
install(FILES ${IPIF_PUBLIC_HDRS} DESTINATION "${INSTALL_INC_DIR}" CONFIGURATIONS Debug)
install(FILES ${SOURCES_LOOP} DESTINATION "${INSTALL_SRC_DIR}" CONFIGURATIONS Debug)
install(FILES ${SOURCES_ZERO} DESTINATION "${INSTALL_SRC_DIR}" CONFIGURATIONS Debug)
