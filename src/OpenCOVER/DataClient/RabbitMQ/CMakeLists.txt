# RabbitMQ client as a separate library

covise_find_package(rabbitmq-c)

if(NOT rabbitmq-c_FOUND)
  message(STATUS "rabbitmq-c not found, skipping RabbitMQ DataClient")
    return()
endif()

# Define sources and headers
SET(HEADERS
  RabbitMQClient.h
)

SET(SOURCES
  RabbitMQClient.cpp
)

# Build the library
ADD_COVISE_LIBRARY(coRabbitMQClient)

# Link against coDataClient (for the base Client interface)
target_link_libraries(coRabbitMQClient coDataClient rabbitmq::rabbitmq)

# Install
COVISE_INSTALL_TARGET(coRabbitMQClient)
COVISE_INSTALL_HEADERS(coRabbitMQClient ${HEADERS})
