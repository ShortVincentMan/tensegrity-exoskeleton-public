# Upper-Limb EMG Trial Logger

## Open and upload

1. Install the PlatformIO IDE extension in VS Code.
2. Select **PlatformIO: Open Project** and choose this folder.
3. Connect the ESP32 and select its serial port if PlatformIO does not detect it.
4. Use **PlatformIO: Upload**.
5. Use **PlatformIO: Serial Monitor**. The configured rate is 921600 baud.

The default board target is `esp32dev`, matching Arduino IDE's generic
**ESP32 Dev Module**. Change `board` in `platformio.ini` if the physical board
uses a different PlatformIO board identifier.

## Trial commands

Set the session once:

```text
session,20260924_VL01
```

Motor-off repetition:

```text
senseonly
trial,M01,MOTOR_OFF,1,0.0
start
end
```

Assisted repetition, after completing the STOP-to-AUTO zeroing sequence:

```text
trial,A01,ASSISTED,1,0.0
start
end
```

Use `rates` to display measured IMU packet rates and controller frequency.
CSV rows begin with `DATA`; diagnostic and event lines begin with `#`.

