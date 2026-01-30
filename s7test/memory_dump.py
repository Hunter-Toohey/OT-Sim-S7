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

# full memory dump of PA area (first 512 bytes)
pa_data = client.read_area(Area.PA, 0, 0, 512)
print("PA Area Dump (first 512 bytes):")
for i in range(0, 512, 16):
    chunk = pa_data[i:i+16]
    hex_chunk = ' '.join(f'{byte:02X}' for byte in chunk)
    print(f"{i:04X}: {hex_chunk}")

# full memory dump of db area (first 512 bytes)
db_data = client.read_area(Area.DB, 1, 0, 512)
print("DB1 Area Dump (first 512 bytes):")
for i in range(0, 512, 16):
    chunk = db_data[i:i+16]
    hex_chunk = ' '.join(f'{byte:02X}' for byte in chunk)
    print(f"{i:04X}: {hex_chunk}") 

# disconnect
client.disconnect()
print("Disconnected")