# Feedback status from DDSM115 motor
float64 current             # Feedback current in Amperes
float64 velocity            # Feedback velocity in RPM (revolutions per minute)
float64 position            # Feedback single-turn position in Degrees (0.0 to 360.0)
float64 winding_temperature # Winding temperature in Celsius
uint8 error_code            # Error flags from the motor controller (bitmask)
