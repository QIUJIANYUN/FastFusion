########################
# LinkAzureKinect.cmake #
########################
IF(WITH_AZUREKINECT)
    TARGET_LINK_LIBRARIES(${targetname} /home/zhuzunjie/3thparty/Azure-Kinect-Sensor-SDK/build/bin/libk4a.so.1.1.0)
ENDIF()