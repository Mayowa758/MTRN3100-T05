Front LiDAR recovery version.
Movement logic is unchanged except for front-obstacle handling.

When front distance <= FRONT_STOP_MM:
- stop
- reverse briefly
- stop and settle
- re-read LiDARs
- continue the same forward command
- do not abort the sequence

Tuning:
REVERSE_PWM = 65
REVERSE_TIME_MS = 250
REVERSE_SETTLE_MS = 120

LiDAR mapping:
LEFT=A0/0x30
RIGHT=A1/0x31
FRONT=A2/0x32
