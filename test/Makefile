# Build tests.

CC = g++
CPPFLAGS = -std=c++20 -Wall -DBOOST_TEST_DYN_LINK
TARGET = tMatrix tKalmanFilter

MY_BOOST_DIR = /usr/local/lib/boost_1_75_0
INCLUDES = -I$(MY_BOOST_DIR) -I../include/
LIBS = -lboost_unit_test_framework

all: tMatrix tKalmanFilter

TARGET1 = tMatrix
$(TARGET1): $(TARGET1).cpp
	$(CC) $(CPPFLAGS) $(INCLUDES) $(LIBS) $(TARGET1).cpp -o $(TARGET1)

TARGET2 = tKalmanFilter
$(TARGET2): $(TARGET2).cpp
	$(CC) $(CPPFLAGS) $(INCLUDES) $(LIBS) $(TARGET2).cpp -o $(TARGET2)

clean:
	$(RM) $(TARGET1) $(TARGET2)
