# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "C:/Users/Public/kdtree/build/dep_glm-prefix/src/dep_glm")
  file(MAKE_DIRECTORY "C:/Users/Public/kdtree/build/dep_glm-prefix/src/dep_glm")
endif()
file(MAKE_DIRECTORY
  "C:/Users/Public/kdtree/build/dep_glm-prefix/src/dep_glm-build"
  "C:/Users/Public/kdtree/build/dep_glm-prefix"
  "C:/Users/Public/kdtree/build/dep_glm-prefix/tmp"
  "C:/Users/Public/kdtree/build/dep_glm-prefix/src/dep_glm-stamp"
  "C:/Users/Public/kdtree/build/dep_glm-prefix/src"
  "C:/Users/Public/kdtree/build/dep_glm-prefix/src/dep_glm-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/Public/kdtree/build/dep_glm-prefix/src/dep_glm-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/Public/kdtree/build/dep_glm-prefix/src/dep_glm-stamp${cfgdir}") # cfgdir has leading slash
endif()
