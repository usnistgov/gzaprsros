#!/usr/bin/python

import sys, getopt, signal
from pymodbus.client.sync import ModbusTcpClient as ModbusClient

ConveyorOnAddress = 0x8000
ConveyorReverseAddress = 0x8001
ConveyorSpeedAddress = 0x8002

# signal handler for ^C
def quit(sig, frame):
    sys.exit(0)

def writeIt(client, value):
    if value < 0:
        client.write_register(ConveyorReverseAddress, 1)
        client.write_register(ConveyorSpeedAddress, -value);
        client.write_register(ConveyorOnAddress, 1)
    elif value > 0:
        client.write_register(ConveyorReverseAddress, 0)
        client.write_register(ConveyorSpeedAddress, value)
        client.write_register(ConveyorOnAddress, 1)
    else:
        client.write_register(ConveyorSpeedAddress, 0)
        client.write_register(ConveyorOnAddress, 0)

def main():
    """
    Testing application for the APRS conveyer via Modbus.

    Speeds are signed integers, -32768 .. 32767. Positive values move forward,
    negative values move reverse.

    Syntax: aprs_modbus_client.py <args>, where <args> are

    -h <host>, --host=<host> for the IP address or hostname of the Modbus server
    -p <port>, --port=<port> for the TCP port of the Modbus server
    -v <value>, --value=<value> to write a single speed value and quit
    -?, --help to print this help message and quit

    With no <value>, enters a loop on user input for values.
    """

    host = "localhost"
    port = 502
    value = None

    signal.signal(signal.SIGINT, quit)

    try:
        opts, args = getopt.getopt(sys.argv[1:], "h:p:v:?", ["host=", "port=", "value=", "help"])
    except getopt.GetoptError as err:
        print(err)
        sys.exit(1)

    for o, a in opts:
        if o in ("-h", "--host"):
            host = a
        elif o in ("-p", "--port"):
            port = int(a)
        elif o in ("-v", "--value"):
            value = int(a)
        elif o in ("-?", "--help"):
            print(main.__doc__)
            sys.exit(0)
        else:
            print("option " + o + " not recognized")
            sys.exit(1)

    client = ModbusClient(host, port)
    if not client.connect():
        print("can't connect to " + str(host) + " on port " + str(port));
        sys.exit(1)

    # if a value was passed, just write it and quit
    if value is not None:
        writeIt(client, value)
        sys.exit(0)

    # otherwise, loop on input and get lots of values
    while True:
        line = sys.stdin.readline()
        if line == "" : break
        toks = line.split()
        try: cmd = toks[0]
        except: continue # blank line
        try: value = int(cmd)
        except:
            print("bad input: " + cmd + ": need a number");
            continue;
        writeIt(client, value)

    sys.exit(0)

if __name__ == "__main__":
    main()
