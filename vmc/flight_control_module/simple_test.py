#!/usr/bin/env python3
import asyncio
from mavsdk import System
async def run():
    drone = System()
    await drone.connect(system_address="udp://127.0.0.1:14541")
    print("Waiting for drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state.uuid}")
            break
if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())