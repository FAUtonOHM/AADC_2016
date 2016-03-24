# declare our filter
set(FILTER_NAME user_FollowPath)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    stdafx.cpp

    FollowPath.h
    FollowPath.cpp
)

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} aadcuser/follow_path) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
