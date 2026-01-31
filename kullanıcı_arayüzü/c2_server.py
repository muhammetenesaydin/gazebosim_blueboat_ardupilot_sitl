import asyncio
import json
import logging
import time
import math
from pymavlink import mavutil
import websockets

# --- Configuration ---
MAVLINK_CONNECTION = "/dev/ttyUSB0"  # Default for Radio on GCS or ACM0 for Pixhawk
MAVLINK_BAUD = 57600
WEBSOCKET_PORT = 8888

# Logging Setup
logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')
logger = logging.getLogger("C2_Server")

class C2Server:
    def __init__(self, connection_str, baud):
        self.connection_str = connection_str
        self.baud = baud
        self.conn = None
        self.clients = set()
        self.telemetry_data = {
            "lat": 0.0, "lon": 0.0, "alt": 0.0,
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            "heading": 0, "voltage": 0.0, "armed": False,
            "mode": "UNKNOWN", "groundspeed": 0.0,
            "rssi": 0, "remrssi": 0, "noise": 0, "remnoise": 0
        }

    async def connect_mavlink(self):
        while True:
            try:
                logger.info(f"Connecting to MAVLink on {self.connection_str}...")
                self.conn = mavutil.mavlink_connection(self.connection_str, baud=self.baud)
                self.conn.wait_heartbeat(timeout=5)
                logger.info("MAVLink Heartbeat received!")
                break
            except Exception as e:
                logger.error(f"MAVLink Connection failed: {e}. Retrying in 5s...")
                await asyncio.sleep(5)

    async def mavlink_receiver(self):
        """Receive MAVLink messages and update internal state."""
        while True:
            if not self.conn:
                await asyncio.sleep(1)
                continue

            msg = self.conn.recv_match(blocking=False)
            if msg:
                msg_type = msg.get_type()

                if msg_type == 'GLOBAL_POSITION_INT':
                    self.telemetry_data["lat"] = msg.lat / 1e7
                    self.telemetry_data["lon"] = msg.lon / 1e7
                    self.telemetry_data["alt"] = msg.relative_alt / 1000.0
                    self.telemetry_data["heading"] = msg.hdg / 100.0
                
                elif msg_type == 'ATTITUDE':
                    self.telemetry_data["roll"] = math.degrees(msg.roll)
                    self.telemetry_data["pitch"] = math.degrees(msg.pitch)
                    self.telemetry_data["yaw"] = math.degrees(msg.yaw)

                elif msg_type == 'VFR_HUD':
                    self.telemetry_data["groundspeed"] = msg.groundspeed
                
                elif msg_type == 'SYS_STATUS':
                    self.telemetry_data["voltage"] = msg.voltage_battery / 1000.0

                elif msg_type == 'HEARTBEAT':
                    self.telemetry_data["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.telemetry_data["mode"] = mavutil.mode_string_v10(msg)

                elif msg_type == 'RADIO_STATUS':
                    self.telemetry_data["rssi"] = msg.rssi
                    self.telemetry_data["remrssi"] = msg.remrssi
                    self.telemetry_data["noise"] = msg.noise
                    self.telemetry_data["remnoise"] = msg.remnoise

            await asyncio.sleep(0.01)

    async def telemetry_broadcaster(self):
        """Broadcast telemetry to all connected WebSocket clients."""
        while True:
            if self.clients:
                payload = json.dumps({"type": "telemetry", "data": self.telemetry_data})
                await asyncio.gather(*[client.send(payload) for client in self.clients], return_exceptions=True)
            await asyncio.sleep(0.2) # 5Hz

    async def ws_handler(self, websocket):
        """Handle incoming WebSocket messages (commands/missions)."""
        self.clients.add(websocket)
        logger.info(f"New client connected. Total: {len(self.clients)}")
        try:
            async for message in websocket:
                data = json.loads(message)
                cmd_type = data.get("type")

                if cmd_type == "set_mode" and self.conn:
                    mode = data.get("mode")
                    logger.info(f"Setting mode to {mode}")
                    mode_id = self.conn.mode_mapping().get(mode)
                    if mode_id is not None:
                        self.conn.mav.set_mode_send(self.conn.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)

                elif cmd_type == "arm" and self.conn:
                    value = 1 if data.get("value") else 0
                    logger.info(f"Setting Arm status to {value}")
                    self.conn.mav.command_long_send(
                        self.conn.target_system, self.conn.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, value, 0, 0, 0, 0, 0, 0
                    )

                elif cmd_type == "upload_mission" and self.conn:
                    waypoints = data.get("waypoints", [])
                    logger.info(f"Uploading mission with {len(waypoints)} waypoints")
                    await self.upload_mission(waypoints)

        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.remove(websocket)
            logger.info("Client disconnected")

    async def upload_mission(self, waypoints):
        """MAVLink Mission Upload Logic."""
        count = len(waypoints)
        self.conn.mav.mission_count_send(self.conn.target_system, self.conn.target_component, count)
        
        for i in range(count):
            msg = await self.wait_for_msg('MISSION_REQUEST')
            if msg and msg.seq == i:
                wp = waypoints[i]
                self.conn.mav.mission_item_int_send(
                    self.conn.target_system, self.conn.target_component, i,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0, 1, 0, 0, 0, 0,
                    int(wp['lat'] * 1e7), int(wp['lon'] * 1e7), 10.0 # 10m altitude
                )
        
        ack = await self.wait_for_msg('MISSION_ACK')
        if ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            logger.info("Mission accepted by vehicle!")

    async def wait_for_msg(self, type_name, timeout=2):
        start = time.time()
        while time.time() - start < timeout:
            msg = self.conn.recv_match(type=type_name, blocking=False)
            if msg: return msg
            await asyncio.sleep(0.1)
        return None

    async def run(self):
        await self.connect_mavlink()
        async with websockets.serve(self.ws_handler, "0.0.0.0", WEBSOCKET_PORT):
            logger.info(f"WebSocket C2 Server started on port {WEBSOCKET_PORT}")
            await asyncio.gather(
                self.mavlink_receiver(),
                self.telemetry_broadcaster()
            )

if __name__ == "__main__":
    server = C2Server(MAVLINK_CONNECTION, MAVLINK_BAUD)
    try:
        asyncio.run(server.run())
    except KeyboardInterrupt:
        logger.info("Server stopped.")
