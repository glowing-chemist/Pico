cmake_minimum_required(VERSION 3.10)
set(PROJECT_NAME "PICO")
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 20)

set(SOURCE 
	Source/main.cpp
	Source/Core/Image.cpp
	Source/Core/Scene.cpp
	Source/Core/Sampler.cpp
	Source/Core/Camera.cpp

	Source/Render/PBR.cpp

	Source/Util/Options.cpp
	Source/Util/FrameBuffer.cpp

	# used for texture loading
	ThirdParty/stb_image/stb_image.cpp
	ThirdParty/stb_image/stbi_image_write.cpp
	ThirdParty/glad/src/glad.c
	)


# avoid building unnecessary stuff
set(GLFW_BUILD_DOCS 0)
set(GLFW_BUILD_TESTS 0)
set(GLFW_BUILD_EXAMPLES 0)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/ThirdParty/GLFW3/glfw-3.2.1")
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/ThirdParty/assimp-4.1.0")

add_definitions(-DGLM_ENABLE_EXPERIMENTAL)

include_directories(
			"${CMAKE_CURRENT_LIST_DIR}/"
			"${CMAKE_CURRENT_LIST_DIR}/Source/"

			"${CMAKE_CURRENT_LIST_DIR}/ThirdParty/GLFW3/glfw-3.2.1/include"
			"${CMAKE_CURRENT_LIST_DIR}/ThirdParty/glm"
			"${CMAKE_CURRENT_LIST_DIR}/ThirdParty/assimp-4.1.0/include"
			"${CMAKE_CURRENT_LIST_DIR}/ThirdParty/imgui-master"
			"${CMAKE_CURRENT_LIST_DIR}/ThirdParty/stb_image"
			"${CMAKE_CURRENT_LIST_DIR}/ThirdParty/glad/include"
			)

add_executable(Pico ${SOURCE})
target_link_libraries(Pico glfw)
