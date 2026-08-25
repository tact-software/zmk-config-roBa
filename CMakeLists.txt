if(CONFIG_ZMK_BEHAVIOR_MKP_QUICK_EXIT)
  zephyr_library_named(behavior_mkp_quick_exit)
  zephyr_library_sources(src/behaviors/behavior_mkp_quick_exit.c)
endif()

if(CONFIG_ZMK_INPUT_PROCESSOR_TRACKBALL_GESTURE)
  zephyr_library_named(input_processor_trackball_gesture)
  zephyr_library_sources(src/input_processors/input_processor_trackball_gesture.c)
endif()

if(CONFIG_ZMK_INPUT_PROCESSOR_MOTION_NOTE)
  zephyr_library_named(input_processor_motion_note)
  zephyr_library_sources(src/input_processors/input_processor_motion_note.c)
endif()
