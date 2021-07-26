# VRC-2021-Phase-II

## MQTT Topics

For each topic in the list, the sublist are the keys available in the dictionary
that is sent on the topic.

This is a list of topics you can subscribe to, to obtain telemetry information.

- vrc/battery
  - "voltage": Battery voltage
  - "soc": State of charge, aka percent full
  - "timestamp": Time the message was sent in [ISO 8601 format](https://docs.python.org/3/library/datetime.html#datetime.datetime.isoformat)
- vrc/status
  - "armed": True/False if the drone is currently armed
  - "mode": Current flight mode, which is one of the following:
    - "UNKNOWN"
    - "READY"
    - "TAKEOFF"
    - "HOLD"
    - "MISSION"
    - "RETURN_TO_LAUNCH"
    - "LAND"
    - "OFFBOARD"
    - "FOLLOW_ME"
    - "MANUAL"
    - "ALTCTL"
    - "POSCTL"
    - "ACRO"
    - "STABILIZED"
    - "RATTITUDE"
  - "timestamp": Time the message was sent in [ISO 8601 format](https://docs.python.org/3/library/datetime.html#datetime.datetime.isoformat)
- vrc/location/local
  - "dX": X position in a local North/East/Down coordinate system
  - "dY": Y position in a local North/East/Down coordinate system
  - "dZ": Z position in a local North/East/Down coordinate system
  - "timestamp": Time the message was sent in [ISO 8601 format](https://docs.python.org/3/library/datetime.html#datetime.datetime.isoformat)
- vrc/location/global
  - "lat": Latitude in global coordinates
  - "lon": Longitude in global coordinates
  - "alt": Relative altitude in global coordinates
  - "hdg": Heading
  - "timestamp": Time the message was sent in [ISO 8601 format](https://docs.python.org/3/library/datetime.html#datetime.datetime.isoformat)
- vrc/location/global
  - "lat": Latitude relative to the home position
  - "lon": Longitude relative to the home position
  - "alt": Relative altitude to the home position
  - "timestamp": Time the message was sent in [ISO 8601 format](https://docs.python.org/3/library/datetime.html#datetime.datetime.isoformat)
- vrc/attitude/euler
  - "roll": Roll in degrees
  - "pitch": Pitch in degrees
  - "yaw": Yaw in degrees
  - "timestamp": Time the message was sent in [ISO 8601 format](https://docs.python.org/3/library/datetime.html#datetime.datetime.isoformat)
- vrc/velocity
  - "vX": X velocity in a local North/East/Down coordinate system
  - "vY": Y velocity in a local North/East/Down coordinate system
  - "vZ": Z velocity in a local North/East/Down coordinate system
  - "timestamp": Time the message was sent in [ISO 8601 format](https://docs.python.org/3/library/datetime.html#datetime.datetime.isoformat)

This is a list of topics you can send data to, to make things happen on the drone:

- vrc/pcc/set_base_color
  - "wrgb": A list of 4 `int`s between 0 and 255 to set the base color of the LEDs. Example: `[0, 128, 255]`.
- vrc/pcc/set_temp_color
  - "wrgb": A list of 4 `int`s between 0 and 255 to set a temporary color of the LEDs. Example: `[0, 128, 255]`.
  - "time": Optional `float` for the number of seconds the color should be set for. Default is `0.5`.
- vrc/pcc/set_servo_open_close
  - "servo": ID of the servo to open or close as a `int`.
  - "action": Either the literal string `"open"` or `"close"`.
- vrc/pcc/set_servo_min
  - "servo": ID of the servo to set the minimum pulse width of as a `int`.
  - "min_pulse": A `float` between 0 and 1000.
- vrc/pcc/set_servo_max
  - "servo": ID of the servo to set the maximum pulse width of as a `int`.
  - "max_pulse": A `float` between 0 and 1000.
- vrc/pcc/set_servo_pct
  - "servo": ID of the servo to set the percent of as a `int`.
  - "percent": A `float` between 0 and 100.
- vrc/pcc/reset
  - There is no payload required for this topic. This will reset all of the peripheals.