target_include_directories(app PRIVATE .)
target_include_directories(app PRIVATE ble_central_role)

FILE(GLOB_RECURSE app_sources *.c)
target_sources(app PRIVATE ${app_sources})