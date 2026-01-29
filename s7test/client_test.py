#!/usr/bin/env python3
# pip install python-snap7 to run this script
# chmod +x * to make this script executable

import snap7
from snap7.type import Area
import struct

# connect to S7 server
client = snap7.client.Client()
client.connect("127.0.0.1", 0, 1, 102)
print(f"Connected: {client.get_connected()}")

ADDRESS = 4
VALUE = 75.0

byte_offset = 256 + ADDRESS

# convert float to big-endian bytes
data = bytearray(4)
struct.pack_into('>f', data, 0, VALUE)

# write to PA memory
client.write_area(Area.PA, 0, byte_offset, data)
print(f"Wrote QW{ADDRESS} (byte {byte_offset}) = {VALUE}")

# read back from PA memory
read_data = client.read_area(Area.PA, 0, byte_offset, 4)
read_value = struct.unpack('>f', read_data)[0]
print(f"Read QW{ADDRESS} (byte {byte_offset}) = {read_value}")

# disconnect
client.disconnect()
print("Disconnected")