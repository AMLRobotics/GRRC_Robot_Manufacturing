#!/usr/bin/env python3

import cantools
import can
 
try:
    # The 'with' statement ensures proper closure
    with can.interface.Bus('vcan0', interface='socketcan') as bus:
        print("CAN 메시지 수신 중... (Ctrl+C로 종료)")
        try:
            # 버스를 반복하며 메시지 수신
            for msg in bus:
                print(f"ID: {msg.arbitration_id:X}, Data: {msg.data.hex()}")
        except KeyboardInterrupt:
            print("수신 종료.")

        # Add other CAN operations here

# The messages will match (except potentially timestamps)
except Exception as e:
    print(f"An error occurred: {e}")



