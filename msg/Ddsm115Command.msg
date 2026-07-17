# Control modes
uint8 MODE_CURRENT = 1
uint8 MODE_VELOCITY = 2
uint8 MODE_POSITION = 3

# Brake modes (for velocity mode)
uint8 BRAKE_RELEASE = 0
uint8 BRAKE_LOCK = 1

uint8 mode        # Control loop mode (MODE_CURRENT, MODE_VELOCITY, or MODE_POSITION)
float64 value     # Command value (Amperes, RPM, or Degrees depending on mode)
uint8 brake_mode  # Brake behavior (BRAKE_RELEASE or BRAKE_LOCK)
