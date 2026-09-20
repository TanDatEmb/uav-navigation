#pragma once

// Keep the installed mapping ABI product-owned. Backend symbols are linked
// privately into the implementation and must not become a supported ABI.
#if defined _WIN32 || defined __CYGWIN__
  #ifdef NAVIGATION_MAPPING_BUILDING_DLL
    #define NAVIGATION_MAPPING_PUBLIC __declspec(dllexport)
  #else
    #define NAVIGATION_MAPPING_PUBLIC __declspec(dllimport)
  #endif
#else
  #define NAVIGATION_MAPPING_PUBLIC __attribute__((visibility("default")))
#endif
