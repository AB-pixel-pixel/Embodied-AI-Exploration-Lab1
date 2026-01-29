cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_maze)

find_package(catkin REQUIRED)

catkin_package()

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 安装 world 文件
install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
