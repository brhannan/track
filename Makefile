# Build a simple test application for KF.

CC = g++
CPPFLAGS = -std=c++11 -Wall
TARGET = mattest

MY_BOOST_DIR = /usr/local/lib/boost_1_75_0
INCLUDES = -I$(MY_BOOST_DIR) -I./include/
# SRCS =

all: $(TARGET)

$(TARGET): $(TARGET).cpp
	$(CC) $(CPPFLAGS) $(INCLUDES) -o $(TARGET) $(TARGET).cpp

clean:
	$(RM) $(TARGET)
