import asyncio
import json
import re
from bleak import BleakScanner, BleakClient
import argparse
import websockets
import aiohttp
import time

CHARACTERISTIC_UUID = 'f22535de-5375-44bd-8ca9-d0ea9ff9e410'
CONTROL_UNIT_NAME = 'QT Py ESP32-S3'

ble_client = None  # Global BLE client

global message_queue


def create_command(addr, mode, duty, freq):
    serial_group = addr // 30
    serial_addr = addr % 30
    byte1 = (serial_group << 2) | (mode & 0x01)
    byte2 = 0x40 | (serial_addr & 0x3F)  # 0x40 represents the leading '01'
    byte3 = 0x80 | ((duty & 0x0F) << 3) | (freq)  # 0x80 represents the leading '1'
    return bytearray([byte1, byte2, byte3])

async def setMotor(client, message):
    try:        
        data_segments = re.findall(r'\{.*?\}', message)
        if not data_segments:
            return  

        commands = []
        for data_segment in data_segments:
            data_parsed = json.loads(data_segment)
            command = create_command(
                data_parsed['addr'], 
                data_parsed['mode'], 
                data_parsed['duty'], 
                data_parsed['freq'] 
            )
            commands.append(command)
        max_chunk_size = 20 
        chunk = bytearray()
        for command in commands:
            if len(chunk) + len(command) <= max_chunk_size:
                chunk += command
            else:
                await client.write_gatt_char(CHARACTERISTIC_UUID, chunk)
                chunk = bytearray(command)  # Start new chunk with current command

        # Send any remaining commands
        if chunk:
            await client.write_gatt_char(CHARACTERISTIC_UUID, chunk)

    except Exception as e:
        print(f'Error in setMotor: {e}')

async def ble_task():
    print('Starting BLE task')
    global ble_client
    while True:
        devices = await BleakScanner.discover()
        for d in devices:
            if d.name and d.name == CONTROL_UNIT_NAME:
                print('Feather device found!')
                ble_client = BleakClient(d.address)
                await ble_client.connect()
                print(f'BLE connected to {d.address}')
                val = await ble_client.read_gatt_char(CHARACTERISTIC_UUID)
                print('Motor read = ', val)
                return  # Exit after successful connection
        await asyncio.sleep(5)  # Retry every 5 seconds

async def handle_connection(websocket):
    global message_queue
    print('WebSocket connection established!')
    message_queue = asyncio.Queue()
    messages = []
    messages_lock = asyncio.Lock()
    # Start background tasks
    asyncio.create_task(collect_messages(websocket, message_queue))
    asyncio.create_task(process_messages(message_queue, messages, messages_lock))
    asyncio.create_task(timer_task(messages, messages_lock))
    # Keep the connection open
    await websocket.wait_closed()

async def collect_messages(websocket, message_queue):
    try:
        async for message in websocket:
            print(f"Received message: {message}")
            await message_queue.put(message)
    except websockets.exceptions.ConnectionClosed as e:
        print(f'WebSocket closed: {e}')
    except Exception as e:
        print(f'Error in collect_messages: {e}')

async def process_messages(message_queue, messages, messages_lock):
    while True:
        message = await message_queue.get()
        print(f"Processing message: {message}")
        messages_to_send = None
        async with messages_lock:
            # Replace message if one with the same 'addr' exists
            message_json = json.loads(message)
            addr = message_json.get('addr')
            for i, existing_message in enumerate(messages):
                existing_message_json = json.loads(existing_message)
                if existing_message_json.get('addr') == addr:
                    messages[i] = message
                    break
            else:
                messages.append(message)
            # If we have 10 messages, process them immediately
            if len(messages) >= 10:
                print(f"More than 10 messages, processing immediately")
                messages_to_send = messages.copy()
                messages.clear()
        if messages_to_send:
            await process_and_send_messages(messages_to_send)

async def timer_task(messages, messages_lock):
    while True:
        timestart = time.time()
       # print(f"Time step: ", timestart)
        await asyncio.sleep(0.02)  # Wait for 20 ms
        messages_to_send = None
        async with messages_lock:
            if messages:
                messages_to_send = messages.copy()
                messages.clear()
        if messages_to_send:
            await process_and_send_messages(messages_to_send)
            print(f"Took {time.time() - timestart} seconds to process and send messages")

async def process_and_send_messages(messages):
    print(f"Processing messages: {messages}")
    combined_message = ''.join(messages)
    if ble_client and ble_client.is_connected:
        #launmc coroutine to send await setMotor(ble_client, combined_message)
        await setMotor(ble_client, combined_message)
        # Launch coroutine to send response to client
        asyncio.create_task(send_to_server(combined_message))
    else:
        print("BLE client not connected yet.")
        asyncio.create_task(send_to_server(combined_message))

async def send_to_server(message):
    async with aiohttp.ClientSession() as session:
            await session.post('http://localhost:5000/commands', json={'command': message, 'timestamp': time.time()})

async def main():
    # Initialize the argument parser
    parser = argparse.ArgumentParser(description="Read CHARACTERISTIC_UUID and CONTROL_UNIT_NAME from the command line.")

    # Add arguments with flags
    parser.add_argument(
        "-uuid", "--characteristic_uuid", required=False, type=str,
        default="f22535de-5375-44bd-8ca9-d0ea9ff9e410",
        help="The UUID of the characteristic"
    )
    
    parser.add_argument(
        "-name", "--control_unit_name", required=False, type=str, 
        default="QT Py ESP32-S3",
        help="The Bluetooth name of the control unit"
    )

    # Parse the arguments
    args = parser.parse_args()

    # Access and print the parameters
    print(f"CHARACTERISTIC_UUID: {args.characteristic_uuid}")
    print(f"CONTROL_UNIT_NAME: {args.control_unit_name}")

    global CHARACTERISTIC_UUID, CONTROL_UNIT_NAME
    CHARACTERISTIC_UUID = args.characteristic_uuid
    CONTROL_UNIT_NAME = args.control_unit_name

    # Start BLE scanning task
    ble_scanning_task = asyncio.create_task(ble_task())

    # Start the WebSocket server
    server = await websockets.serve(handle_connection, 'localhost', 9052)
    print("WebSocket server running on ws://localhost:9052")

    # Keep the server running
    await asyncio.Future()  # Run forever

if __name__ == "__main__":
    asyncio.run(main())
