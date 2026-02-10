#!/usr/bin/env python3

import cantools
import can
 
try:
    # The 'with' statement ensures proper closure
    with can.interface.Bus('vcan0', interface='socketcan') as bus:
        msg = can.Message(arbitration_id=0x123, data=[1, 2, 3, 4, 5, 6])
        bus.send(msg)

        # Add other CAN operations here

# The messages will match (except potentially timestamps)
except Exception as e:
    print(f"An error occurred: {e}")



