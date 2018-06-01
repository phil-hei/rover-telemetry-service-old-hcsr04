PROJECT_TARGET_ADD(rover-telemetry-old)

INCLUDE_DIRECTORIES(include/)

file(GLOB all_cpp_sources "src/*.cpp")

# Define project Targets
ADD_EXECUTABLE(${TARGET_NAME} ${all_cpp_sources})
SET_TARGET_PROPERTIES(${TARGET_NAME} PROPERTIES
	LABELS "EXECUTABLE"
	OUTPUT_NAME ${TARGET_NAME}
)

# Library dependencies (include updates automatically)
TARGET_LINK_LIBRARIES(${TARGET_NAME}
	${link_libraries} rover-services-api roverapi ${libafbwsc_LIBRARIES}
)
