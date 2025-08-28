# Space Station Electrical Power Systems

**Electrical Power Systems (EPS) Simulation for Space Station OS**

This ROS 2 package simulates the electrical power subsystem onboard a space station, focusing on real-time monitoring of battery health and readiness for power distribution.

---

## Features

The current version includes:

- **Battery health status simulation** for 24 ORUs (Orbital Replacement Units), i.e., 2 batteries per channel across 12 EPS channels.
- Each battery has:
  - A unique `battery_bms_<ID>` identifier.
  - A dedicated publisher for health status (`sensor_msgs/BatteryState`).
  - A service server (`std_srvs/Trigger`) for discharge requests (used by BCDU and MBSU).
- Battery **location mapping** (e.g., `channel_1`, `channel_2`, ...) is done via a YAML config file.
- **Per-battery diagnostics** published on `/eps/diagnostics` when operating outside expected voltage range.

---

## Published Topics

Each battery publishes its health to a dedicated topic:

```

/battery/battery\_bms\_<ID>/health

```

Example topics:

```

/battery/battery\_bms\_0/health
/battery/battery\_bms\_1/health
...
/battery/battery\_bms\_23/health
/eps/diagnostics

````

---

## Battery Status Message

The topic messages conform to the standard [`sensor_msgs/BatteryState`](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/BatteryState.msg) and include fields like:

```yaml
header:
  frame_id: battery_bms_18
voltage: 120.0
current: 4.0
charge: 100.0
capacity: 100.0
percentage: 1.0
power_supply_status: 1         # Charging
power_supply_health: 1         # Good
power_supply_technology: 2     # Li-Ion
present: true
cell_voltage: [38 values]
cell_temperature: [38 values]
location: channel_10
serial_number: battery_bms_18
````

Values are updated every second and include:

* Realistic simulation of 38-cell voltage and temperature profiles
* Charging/discharging state changes based on service requests

---

## Parameters

This node uses the following parameters:

```yaml
/battery_manager_node:
  ros__parameters:
    num_channels: 12
    battery_config: "config/battery_config.yaml"
```

Example YAML mapping for battery locations:

```yaml
battery_bms_0: channel_1
battery_bms_1: channel_1
...
battery_bms_23: channel_12
```

---

## Future Work

* Diagnostic failure modes (e.g., low voltage, overheat, dead battery) will be added in future versions.
* Integration with the **Battery Charge/Discharge Unit (BCDU)** and **Main Bus Switching Unit (MBSU)**.
* Aggregated per-channel monitoring and EPS fault reporting.

---


**Maintainer**: Siddarth Dayasagar
**License**: \[Apache 2.0 or your choice]

```
