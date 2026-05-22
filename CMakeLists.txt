if(CONFIG_ZMK_BEHAVIOR_MKP_QUICK_EXIT)
  zephyr_library_named(behavior_mkp_quick_exit)
  zephyr_library_sources(src/behaviors/behavior_mkp_quick_exit.c)
endif()
