######################
# UseAzureKinect.cmake #
######################

OPTION(WITH_AZUREKINECT "Build with Microsoft Azure Kinect support?" ON)

IF(WITH_AZUREKINECT)
    include_directories(/home/zhuzunjie/3thparty/Azure-Kinect-Sensor-SDK/include/)
    include_directories(/home/zhuzunjie/3thparty/Azure-Kinect-Sensor-SDK/build/src/sdk/include/)
    ADD_DEFINITIONS(-DCOMPILE_WITH_Azurekinect)
ENDIF()
