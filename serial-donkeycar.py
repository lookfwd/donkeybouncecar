import serial
import sys
import asyncio
import websockets
import json
import math
import time

# Based on https://websockets.readthedocs.io/en/stable/intro.html

port = "/dev/tty.usbmodem14501"

s1 = serial.Serial(port, 115200)
s1.flushInput()

USERS = set()

async def serve_user(websocket, path):
    USERS.add(websocket)
    try:
        async for message in websocket:
            data = json.loads(message)
            pass
    finally:
        USERS.remove(websocket)

def dist(a,b):
    return math.sqrt((a*a)+(b*b))

async def periodic():
    RATIO = 50
    start = time.time()
    parse_cnt = 0
    buf = []
    while True:
        try:
            while s1.inWaiting() > 0:
                inputValue = s1.read(1).decode('ascii')
                buf.append(inputValue)
                if inputValue == '\n':
                    line = ("".join(buf)).strip()
                    info = line.split('|')

                    # http://aprs.gids.nl/nmea/#gga
                    # Example: "4043.4476,N,07359.6348,W,1,6"
                    nmea = info[9].split(',')

                    if len(nmea) != 6:
                        q = 0
                        nmea = [None, None, None, None, None, None]
                    else:
                        # GPS Quality indicator (0=no fix, 1=GPS fix, 2=Dif. GPS fix)
                        q = int(nmea[4])
                        if q > 0:
                            nmea[0] = float(nmea[0])  # Lat
                            nmea[2] = float(nmea[2])  # Lon
                            nmea[5] = int(nmea[5])  # Satellites

                    # From https://bit.ly/2yOgljE
                    x = int(info[2]) / 16384.0
                    y = int(info[3]) / 16384.0
                    z = int(info[4]) / 16384.0
                    x_rotation = math.degrees(math.atan2(y, dist(x,z)))
                    y_rotation = -math.degrees(math.atan2(x, dist(y,z)))

                    info = {"Steering": int(info[0]),
                            "Throttle": int(info[1]),
                            "XRot": x_rotation,
                            "YRot": y_rotation,
                            "AcX": int(info[2]),
                            "AcY": int(info[3]),
                            "AcZ": int(info[4]),
                            "GyX": int(info[5]),
                            "GyY": int(info[6]),
                            "GyZ": int(info[7]),
                            "SonarCm": int(info[8]),
                            "Lat": nmea[0],
                            "NS": nmea[1],
                            "Lon": nmea[2],
                            "EW": nmea[3],
                            "Q": q,
                            "Sats": nmea[5],
                        }

                    message = json.dumps(info)
                    if USERS:
                        await asyncio.wait([user.send(message) for user in USERS])
                    buf = []

                    parse_cnt += 1
                    if parse_cnt == RATIO:
                        # Reference parses per second without sonar and GPS: 25
                        # With GPS, parses per second: 20
                        # With GPS and sonar parses per second: 15
                        parse_cnt = 0
                        now = time.time()
                        elapsed = now - start
                        print(f"Parses per second: {1/(elapsed/RATIO)}")
                        start = now

            await asyncio.sleep(0.1)
        except:
            import traceback
            traceback.print_exc()

start_server = websockets.serve(serve_user, 'localhost', 5678)

asyncio.get_event_loop().run_until_complete(start_server)
task = asyncio.get_event_loop().create_task(periodic())
def stop():
    task.cancel()
#asyncio.get_event_loop().call_later(5, stop)

asyncio.get_event_loop().run_forever()
