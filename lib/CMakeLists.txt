if (NOT TARGET sparsepp)
  add_library(sparsepp INTERFACE) # v1.21
  target_include_directories(sparsepp INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/sparsepp)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL "8.1.0")
    target_compile_options(sparsepp INTERFACE -Wno-class-memaccess)
  endif()
endif()

if (NOT TARGET ez)
  add_library(ez INTERFACE) # v1.0
  target_include_directories(ez INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/ez)
endif()

if (NOT TARGET fmt)
  add_library(fmt INTERFACE)
  target_include_directories(fmt INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/fmt)
  target_compile_definitions(fmt INTERFACE FMT_HEADER_ONLY)
endif()

if (NOT TARGET glucose)
  add_library(glucose INTERFACE)
  target_include_directories(glucose INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/glucose/include)
endif()

if (NOT TARGET kitty)
  add_library(kitty INTERFACE) # v0.4
  target_include_directories(kitty INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/kitty)
endif()

if (NOT TARGET rang)
  add_library(rang INTERFACE)
  target_include_directories(rang INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/rang)
endif()

if (NOT TARGET lorina)
  add_library(lorina INTERFACE) # v0.1
  target_include_directories(lorina INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/lorina)
  target_link_libraries(lorina INTERFACE rang fmt)
endif()

if (NOT TARGET percy)
  add_library(percy INTERFACE) # >v0.1.2
  target_include_directories(percy INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/percy)
  target_compile_definitions(percy INTERFACE DISABLE_NAUTY)

  set(THREADS_PREFER_PTHREAD_FLAG ON)
  find_package(Threads REQUIRED)
  target_link_libraries(percy INTERFACE Threads::Threads)

  set(ABC_USE_NAMESPACE "pabc")
  set(STATIC_LIBABC true)
  add_subdirectory(abcsat)
  if (UNIX)
    target_compile_definitions(libabcsat PUBLIC "LIN64" ABC_NAMESPACE=pabc ABC_NO_USE_READLINE)
  elseif(WIN32)
    target_compile_definitions(libabcsat PUBLIC ABC_NAMESPACE=pabc ABC_USE_NO_READLINE NOMINMAX)
    target_compile_definitions(libabcsat PUBLIC WIN32_NO_DLL)
  endif()
  target_link_libraries(percy INTERFACE libabcsat)
endif()

if (NOT TARGET mockturtle) # spectral c2e4149
  add_library(mockturtle INTERFACE)
  target_include_directories(mockturtle INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/mockturtle)
  target_link_libraries(mockturtle INTERFACE ez kitty lorina sparsepp percy)
endif()

if (NOT TARGET easy)
  add_library(easy INTERFACE)
  target_include_directories(easy INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/easy)
  target_link_libraries(easy INTERFACE glucose)
endif()

if (NOT TARGET tweedledum)
  add_library(tweedledum INTERFACE)
  target_include_directories(tweedledum INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/tweedledum)
  target_link_libraries(tweedledum INTERFACE easy fmt lorina kitty)
endif()

