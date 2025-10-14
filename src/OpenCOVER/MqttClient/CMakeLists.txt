return()
covise_find_package(PahoMqttCpp QUIET)

if(NOT PahoMqttCpp_FOUND)
     MESSAGE("PahoMqttCpp not found, not building MQTT client library")
    return()
endif()

SET(HEADERS
  MqttClient.h
)

SET(SOURCES
  MqttClient.cpp
)

ADD_COVISE_LIBRARY(coMqttClient)
TARGET_LINK_LIBRARIES(coMqttClient PahoMqttCpp::paho-mqttpp3 nlohmann_json::nlohmann_json coDataClient)
 # ensure the underlying C library is also linked on Windows (name may vary)
  if(TARGET PahoMqttCpp::paho-mqtt3c)
    message("Linking coMqttClient with PahoMqttCpp::paho-mqtt3c")
      target_link_libraries(coMqttClient PahoMqttCpp::paho-mqtt3c)
  endif()

COVISE_INSTALL_TARGET(coMqttClient)
COVISE_INSTALL_HEADERS(coMqttClient ${HEADERS})


COVISE_WNOERROR(coMqttClient BASEARCH zebu)

