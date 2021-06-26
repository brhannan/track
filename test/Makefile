# Build tests.

CC = g++
CPPFLAGS = -std=c++20 -Wall -DBOOST_TEST_DYN_LINK
TARGET = tMatrix tKalmanFilter tExtendedKalmanFilter

MY_BOOST_DIR = /usr/local/lib/boost_1_75_0 # Set boost directory here.
INCLUDES = -I$(MY_BOOST_DIR) -I../include/
LIBS = -lboost_unit_test_framework

all: tMatrix tKalmanFilter tExtendedKalmanFilter

TARGET1 = tMatrix
$(TARGET1): $(TARGET1).cpp
	$(CC) $(CPPFLAGS) $(INCLUDES) $(LIBS) $(TARGET1).cpp -o $(TARGET1)

TARGET2 = tKalmanFilter
$(TARGET2): $(TARGET2).cpp
	$(CC) $(CPPFLAGS) $(INCLUDES) $(LIBS) $(TARGET2).cpp -o $(TARGET2)

TARGET3 = tExtendedKalmanFilter
$(TARGET3): $(TARGET3).cpp
	$(CC) $(CPPFLAGS) $(INCLUDES) $(LIBS) $(TARGET3).cpp -o $(TARGET3)

clean:
	$(RM) $(TARGET1) $(TARGET2) $(TARGET3)
