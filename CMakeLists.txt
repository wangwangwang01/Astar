 cmake_minimum_required(VERSION 3.2)
# 增加PCL库的依赖
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io )

# 增加opencv的依赖
FIND_PACKAGE( OpenCV REQUIRED )
include_directories(include)
# 添加头文件和库文件

ADD_EXECUTABLE( Astar src/Astar.cpp )
TARGET_LINK_LIBRARIES( Astar ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} 
)
