add_library( myslam SHARED
    frame.cpp
    mappoint.cpp
    map.cpp
    camera.cpp
    config.cpp
    visual_odometry.cpp
    g2o_types.cpp
    orbextractor.cpp
)

target_link_libraries( myslam
    ${THIRD_PARTY_LIBS}
)
