#!/usr/bin/env python3
import json

# Read the configuration from the json file
config_file_name = "config_dog.json"
json_file = open(config_file_name)
config_data = json.load(json_file)

a = config_data["QUALISYS"]["IP_MOCAP_SERVER"]
print(a)
