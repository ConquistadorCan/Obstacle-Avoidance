import math

from config.path_utils import MISSIONS_DIR

def convert_mission_file_local_to_wgs(mission_file_name: str, output_mission_file_name: str, home_lat: float, home_lon: float):
    def _local_to_gps(x: float, y: float, home_lat: float, home_lon: float) -> tuple:
        lat = home_lat + (y / 111320)
        lon = home_lon + (x / (111320 * math.cos(math.radians(home_lat))))
        return lat, lon

    input_path = MISSIONS_DIR / f"{mission_file_name}.txt"
    output_path = MISSIONS_DIR / f"{output_mission_file_name}.txt"

    with input_path.open("r") as infile, output_path.open("w") as outfile:
        for line in infile:
            if not line.strip():
                continue
            x_str, y_str, z_str = line.strip().split(",")
            lat, lon = _local_to_gps(float(x_str), float(y_str), home_lat, home_lon)
            outfile.write(f"{lat:.7f},{lon:.7f},{float(z_str):.2f}\n")
        
        lat, lon, z_str = 0, 0, 0
        outfile.write(f"{lat:.7f},{lon:.7f},{float(z_str):.2f}\n")
