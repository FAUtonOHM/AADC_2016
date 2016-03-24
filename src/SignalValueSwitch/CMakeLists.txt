# declare our filter
set(FILTER_NAME user_SignalValueSwitch)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    stdafx.cpp

    SignalValueSwitch.h
    SignalValueSwitch.cpp
)

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} aadcuser/signal_value_switch) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
