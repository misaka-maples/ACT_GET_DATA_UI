import os,json

config_file_path = os.path.join(os.path.dirname(__file__), "../config/multi_device_sync_config.json")

multi_device_sync_config = {}
def read_config(config_file: str):
    global multi_device_sync_config
    with open(config_file, "r") as f:
        config = json.load(f)
    for device in config["devices"]:
        multi_device_sync_config[device["serial_number"]] = device
        # print(f"Device {device['serial_number']}: {device['config']['mode']}")


if __name__ == "__main__":
    read_config(config_file_path)
    print(multi_device_sync_config.items())
    serial_number_list = [device["serial_number"] for device in multi_device_sync_config.values()]
    serial_number_list=[ 'CP1L44P0006E','CP1E54200056', 'CP1L44P0004Y']
    camera_index_map = {device['config']['camera_name']: serial_number_list.index(device["serial_number"]) for device in multi_device_sync_config.values() if device["serial_number"] in serial_number_list}
    print(camera_index_map)
