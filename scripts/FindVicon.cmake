include(FindPackageHandleStandardArgs)

FIND_PATH(VICON_INCLUDE_DIR Client.h PATHS /opt/ViconDataStreamSDK)
FIND_LIBRARY(VICON_LIBRARY NAMES ViconDataStreamSDK_CPP PATHS /opt/ViconDataStreamSDK)

# Handle the REQUIRED argument and set the <UPPERCASED_NAME>_FOUND variable
find_package_handle_standard_args(Vicon "Could NOT find Vicon. Please continue." VICON_LIBRARY VICON_INCLUDE_DIR)

if(VICON_FOUND)
    find_package_message(VICON_FOUND "Found Vicon SDK  ${VICON_LIBRARY}" "[${VICON_LIBRARY}][${VICON_INCLUDE_DIR}]")
endif(VICON_FOUND)

mark_as_advanced(VICON_INCLUDE_DIR VICON_LIBRARY)
