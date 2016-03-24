# declare our filter
set(FILTER_NAME user_CarPose)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    stdafx.cpp

    CarPose.h
    CarPose.cpp
)

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} aadcuser/car_pose) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
