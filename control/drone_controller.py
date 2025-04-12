from pymavlink import mavutil

from core.flight_mode_enum import FlightModeEnum

class DroneController:
    def __init__(self, connection_string: str = "tcp:127.0.0.1:5763"):
        self.connection = mavutil.mavlink_connection(connection_string)

    def set_mode(self, mode: FlightModeEnum):
        def _wait_for_mode(mode: FlightModeEnum, timeout_per_attempt: int = 0.05, attempt_limit: int = 5):
            attempt = 0

            while attempt < attempt_limit:
                msg = self.connection.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout_per_attempt)

                if msg is None:
                    attempt += 1
                    continue
                
                if msg.get_srcSystem() != self.connection.target_system: # Heartbeat message came from a different system, 2 means drone
                    attempt += 1
                    continue

                if msg.custom_mode == mode.value:
                    return True
                else:
                    attempt += 1
                    continue
            
            raise Exception(f"Failed to set mode {mode.value} after {attempt_limit} attempts.")
        
        self.connection.mav.set_mode_send(
            self.connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode.value
        )

        _wait_for_mode(mode)

if __name__ == "__main__":
    try:
        drone_controller = DroneController()
        drone_controller.set_mode(FlightModeEnum.AUTO)
    except Exception as e:
        print(e)