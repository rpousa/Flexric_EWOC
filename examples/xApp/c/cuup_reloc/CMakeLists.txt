add_executable(xapp_cuup_reloc_trigger
  xapp_cuup_reloc_trigger.c
  )

target_link_libraries(xapp_cuup_reloc_trigger
                      PUBLIC
                      e42_xapp
                      -pthread
                      -lsctp
                      -ldl
                      )
