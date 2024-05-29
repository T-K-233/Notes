# CMake Best Practices



## In a Nutshell

* Declare modules with `ADD_LIBRARY` or `ADD_EXECUTABLE`
* Declare build flags with `TARGET_xxx()`
* Declare dependencies with `TARGET_LINK_LIBRARIES`
* Specify which is `PUBLIC` and which is `PRIVATE`

## Boilderplate



### Headers

```cmake
cmake_minimum_required(VERSION 3.0)

# declare top-level flags
if (MSVC)
  add_compile_options(/W3 /WX)
else()
  add_compile_options(-W -Wall -Werror)
endif()

```

### Add Libraries

```cmake
add_library(mylib
  src/file1.c
  src/file2.c
  ...)
    
```

### Declare Flags

```cmake
target_include_directories(mylib PUBLIC include)
target_include_directories(mylib PRIVATE src)

if (SOME_SETTING)
  target_compile_definitions(mylib
    PUBLIC WITH_SOME_SETTINGS)
endif()

```

### Declare Dependencies

```cmake
# for public (interface) dependencies
target_link_libraries(mylib PUBLIC abc)

# for private (implementation) dependencies
target_link_libraries(mylib PRIVATE xyz)
```

### Header-only libraries

```cmake
add_library(mylib INTERFACE)

target_include_directories(mylib INTERFACE include)

target_link_libraries(mylib INTERFACE Boost::Boost)
```

INTERFACE basically indicates that we don't need to build anything



## DO NOT

1. Don't use macros that affect all targets:
   * `INCLUDE_DIRECTORIES()`
   * `ADD_DEFINITIONS()`
   * `LINK_LIBRARIES`
2. Dont' use `TARGET_INCLUDE_LIBRARIES()` with path outside your own module
3. Don't use `TARGET_LINK_LIBRARIES()` without specifying scope (PUBLIC, PRIVATE, or INTERFACE)
4. Don't use `TARGET_COMPILE_OPTIONS()` to set flags that affect the ABI

















