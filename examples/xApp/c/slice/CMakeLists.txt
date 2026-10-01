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

# param_slice_moni.c has never been through a compiler: it is written against an
# xApp API this FlexRIC does not have -- a context argument on sm_cb, and value
# rather than pointer access into the RC RAN-parameter structures. Off by default
# so that ran-build does not depend on it; -DBUILD_PARAM_SLICE_MONI=ON to work on
# it.
option(BUILD_PARAM_SLICE_MONI "Build the parameterised slice monitor xApp" OFF)
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
