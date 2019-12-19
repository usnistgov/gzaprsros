# APRS Objects

"APRS Objects" is a set of ROS packages and utilities for interfacing to the objects in the APRS simulation. This includes items such as gears, part trays, and kit trays, and resources such as the light curtains and conveyor.

## Reporting of Physical Objects

The reporting of physical objects is done by the `aprs_objects_node`
application. It is a ROS node that reads model states from a Gazebo
simulation, and sends location information for parts, part trays, and
kit trays to the APRS control system. The node serves two ports, one for each
camera, with objects checked against camera field of view to decide
which camera port to use.

You can trigger a re-read of the configuration parameters for debug
and others by changing them using rosparam set and then publishing
any string on the `aprs_objects_cfg topic`, e.g., 
```
rosparam set /aprs_objects/debug 0xFF
rostopic pub -1 /aprs_objects/aprs_objects_cfg std_msgs/String a
```

Run it using the ROS launch file like this:
```
roslaunch aprs_objects aprs_objects.launch
```

## Safety System

The reading and reporting of breaks in the safety light curtain is done by the "aprs_safety_node" application. 

## Conveyor

Control of the conveyer through a Modbus server is done by the "aprs_modbus_server_node" application. For the Gazebo simulation, a Modbus server provides external access via Modbus clients to the conveyer speed topic.

### Installing the Modbus server library, `libmodbus`

The open-source `libmodbus` C implementation of the Modbus protocol is used as the basis for the `aprs_modbus_server_node`. The `random_test_server.c` general-purpose server was adapted to call out the Modbus write-single-register command, which is mapped to writing conveyer speed on the Gazebo simulation ROS topic. 

You can get `libmodbus` from <https://github.com/stephane/libmodbus>. To install it, clone the repository, configure, compile, and install it: 

```
git clone https://github.com/stephane/libmodbus.git
cd libmodbus
./autogen.sh
./configure
make
make install
```

### Installing the Modbus Python package, `pymodbus`

The Python `pymodbus` package is used for the client testing program `aprs_modbus_client.py`. You can get `pymodbus` from <https://github.com/riptideio/pymodbus>. To install it, clone the repository, then install it:

```
git clone https://github.com/riptideio/pymodbus.git
cd pymodbus
python setup.py install
```

### Running the Modbus Test Client, `aprs_modbus_client.py`

You can run the client testing program following the built-in help:
```
Testing application for the APRS conveyer via Modbus.

Speeds are signed integers, -32768 .. 32767. Positive values move forward,
negative values move reverse.

Syntax: aprs_modbus_client.py <args>, where <args> are

-h <host>, --host=<host> for the IP address or hostname of the Modbus server
-p <port>, --port=<port> for the TCP port of the Modbus server
-v <value>, --value=<value> to write a single speed value and quit
-?, --help to print this help message and quit

With no <value>, enters a loop on user input for values.
```
In the `aprs_objects` package, you run it like this: 
```
rosrun aprs_objects aprs_modbus_client.py -p 1502
```
