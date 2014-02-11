# location of the Python header file
PYTHON_VERSION = 2.6
PYTHON_INCLUDE = /usr/include/python$(PYTHON_VERSION)
PYTHON_LIB = python$(PYTHON_VERSION)

ROBOARD_INCLUDE = /home/blimp/Desktop/playground/roboard/libsrc
ROBOARD_LIB_DIR = /home/blimp/Desktop/playground/roboard
ROBOARD_LIB = RBIO

BOOST_INC = /usr/include
BOOST_LIB = /usr/lib

ALL_LIB = -L$(BOOST_LIB) -lboost_python -L$(ROBOARD_LIB_DIR) -l$(ROBOARD_LIB) -lpython$(PYTHON_VERSION)
ALL_INCLUDE = -I$(PYTHON_INCLUDE) -I$(BOOST_INC) -I$(ROBOARD_INCLUDE)

TARGET = boost_accelerometer_sensor

# .so file is the file that python will import as a module
$(TARGET).so: $(TARGET).o
	g++ -shared -Wl,--export-dynamic $(TARGET).o $(ALL_LIB) -o $(TARGET).so

$(TARGET).o: $(TARGET).cpp
	g++ $(ALL_INCLUDE) -fPIC -c $(TARGET).cpp
