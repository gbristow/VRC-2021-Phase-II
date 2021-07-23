# VRC-2021-Phase-II

## MQTT Topics

For each topic in the list, the sublist are the keys available in the dictionary
that is sent on the topic.

// TODO timestamp formats, can't send datetime objects

- vrc/battery
  - "voltage": Battery voltage
  - "soc": State of charge, aka percent full
  - "timestamp": Time the message was sent
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
  - "timestamp": Time the message was sent
- vrc/location/local
  - "dX": X position in a local North/East/Down coordinate system
  - "dY": Y position in a local North/East/Down coordinate system
  - "dZ": Z position in a local North/East/Down coordinate system
  - "timestamp": Time the message was sent
- vrc/location/global
  - "lat": Latitude in global coordinates
  - "lon": Longitude in global coordinates
  - "alt": Relative altitude in global coordinates
  - "hdg": Heading
  - "timestamp": Time the message was sent
- vrc/location/global
  - "lat": Latitude relative to the home position
  - "lon": Longitude relative to the home position
  - "alt": Relative altitude to the home position
  - "timestamp": Time the message was sent
- vrc/attitude/euler
  - "roll": Roll in degrees
  - "pitch": Pitch in degrees
  - "yaw": Yaw in degrees
  - "timestamp": Time the message was sent
- vrc/velocity
  - "vX": X velocity in a local North/East/Down coordinate system
  - "vY": Y velocity in a local North/East/Down coordinate system
  - "vZ": Z velocity in a local North/East/Down coordinate system
  - "timestamp": Time the message was sent