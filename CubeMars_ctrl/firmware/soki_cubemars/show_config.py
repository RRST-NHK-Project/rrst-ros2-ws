Import("env")
import os
import re

config_path = os.path.join(env["PROJECT_DIR"], "src", "config.hpp")

device_id = "unknown"
mode = "unknown"

if os.path.exists(config_path):
    with open(config_path) as f:
        for line in f:
            line = line.strip()

            m = re.match(r"#define\s+DEVICE_ID\s+(0x[0-9A-Fa-f]+|\d+)", line)
            if m:
                device_id = m.group(1)

            m = re.match(r"#define\s+MODE_(\w+)", line)
            if m:
                mode = m.group(1)

print("")
print("========================================")
print(" Board Config Info")
print("----------------------------------------")
print(" DEVICE_ID :", device_id)
print(" MODE      :", mode)
print("========================================")
print("")