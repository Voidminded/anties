

cmake_minimum_required(VERSION 2.6)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include(FindPkgConfig)
pkg_search_module( STG REQUIRED stage )

ADD_LIBRARY( basic MODULE basic.cc )

# expand annoying colon-separated lists into usable strings

foreach(f ${STG_CFLAGS})
     set (STG_CFLAGS_STR "${STG_CFLAGS_STR} ${f}")
 endforeach(f)

foreach(f ${STG_LDFLAGS})
     set (STG_LDFLAGS_STR "${STG_LDFLAGS_STR} ${f}")
endforeach(f)


set_target_properties( basic PROPERTIES
 COMPILE_FLAGS "${STG_CFLAGS_STR}"
 #LINK_FLAGS "${STG_LDFLAGS_STR} -framework opengl" 
 LINK_FLAGS "${STG_LDFLAGS_STR}" 
)

# delete the "lib" prefix from the plugin libraries
SET_TARGET_PROPERTIES( basic PROPERTIES PREFIX "" )

