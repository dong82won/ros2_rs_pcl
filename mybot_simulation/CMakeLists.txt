cmake_minimum_required(VERSION 3.22)
project(mybot_simulation)

# C++20 표준 준수
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

find_package(ament_cmake REQUIRED)

# ==============================================================================
# 1. 시뮬레이션 정적 에셋 및 설정 디렉터리 설치
# ==============================================================================
install(
  DIRECTORY
    config
    launch
    models
    worlds
  DESTINATION share/${PROJECT_NAME}
)

# ==============================================================================
# 3. 패키지 인덱스 및 배포
# ==============================================================================
ament_package()
