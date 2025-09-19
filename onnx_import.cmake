if(NOT TARGET onnxruntime::onnxruntime)
  add_library(onnxruntime::onnxruntime SHARED IMPORTED GLOBAL)
  set_target_properties(onnxruntime::onnxruntime PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "/home/anast/Requirements/onnxruntime-linux-x64-gpu-1.16.3/include"
    IMPORTED_LOCATION "/home/anast/Requirements/onnxruntime-linux-x64-gpu-1.16.3/lib/libonnxruntime.so")
endif()

