# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS 
"main.cpp"
"Mpu9250.cpp"
"SPIbus.cpp"
"Messagehandler.cpp"
"MadgwickAHRS.c"
"PID.cpp"
)
set(COMPONENT_ADD_INCLUDEDIRS "" "/usr/include/eigen3")

register_component()
