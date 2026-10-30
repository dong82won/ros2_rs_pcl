cmake_minimum_required(VERSION 3.22)
project(mybot_description)

# Modern ament_cmake 빌드 도구 로드
find_package(ament_cmake REQUIRED)


# 디렉토리 설치
install(DIRECTORY
  launch
  rviz
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}
)

# ==============================================================================
# 코드 품질 및 패키징 테스트 (ament 린터 자동 연동)
# ==============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


# 패키지 설정 종료
ament_package()
