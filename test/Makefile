# Build tests.

# CC = g++
# CPPFLAGS = -std=c++11 -Wall -DBOOST_TEST_DYN_LINK
# TARGET = tMatrix
#
# MY_BOOST_DIR = /usr/local/lib/boost_1_75_0
# INCLUDES = -I$(MY_BOOST_DIR) -I../include/
# LIBS = -lboost_unit_test_framework
#
# all: $(TARGET)
#
# $(TARGET): $(TARGET).cpp
# 	$(CC) $(CPPFLAGS) $(INCLUDES) $(LIBS) -o $(TARGET) $(TARGET).cpp
#
# clean:
# 	$(RM) $(TARGET)

CC = g++
CPPFLAGS = -std=c++11 -Wall -DBOOST_TEST_DYN_LINK
TARGET = tMatrix tKalmanFilter

MY_BOOST_DIR = /usr/local/lib/boost_1_75_0
INCLUDES = -I$(MY_BOOST_DIR) -I../include/
LIBS = -lboost_unit_test_framework

all: tMatrix tKalmanFilter

TARGET1 = tMatrix
$(TARGET1): $(TARGET1).cpp
	$(CC) $(CPPFLAGS) $(INCLUDES) $(LIBS) -o $(TARGET1) $(TARGET1).cpp

TARGET2 = tKalmanFilter
$(TARGET2): $(TARGET2).cpp
	$(CC) $(CPPFLAGS) $(INCLUDES) $(LIBS) -o $(TARGET2) $(TARGET2).cpp

clean:
	$(RM) $(TARGET1) $(TARGET2)
