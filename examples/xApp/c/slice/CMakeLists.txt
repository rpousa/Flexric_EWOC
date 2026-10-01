add_executable(xapp_slice_moni_ctrl 
             xapp_slice_moni_ctrl.c
                ../../../../src/util/alg_ds/alg/defer.c
                ../../../../src/util/time_now_us.c
                )

 target_link_libraries(xapp_slice_moni_ctrl
 						PUBLIC
 						e42_xapp
 						-pthread
 						-lsctp
 						-ldl
 						)

# param_slice_moni.c was written against an xApp API this FlexRIC does not have
# and had never been compiled. Fixed; kept behind a switch so that a future
# breakage here can be taken out of ran-build's way with one word rather than a
# submodule revert: -DBUILD_PARAM_SLICE_MONI=OFF.
option(BUILD_PARAM_SLICE_MONI "Build the parameterised slice monitor xApp" ON)
if(BUILD_PARAM_SLICE_MONI)

  add_executable(param_slice_moni 
               param_slice_moni.c
                  ../../../../src/util/alg_ds/alg/defer.c
                  ../../../../src/util/time_now_us.c
                  )

   target_link_libraries(param_slice_moni
   						PUBLIC
   						e42_xapp
   						-pthread
   						-lsctp
   						-ldl
   						)

endif()
