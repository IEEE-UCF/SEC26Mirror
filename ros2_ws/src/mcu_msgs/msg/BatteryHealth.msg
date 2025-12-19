# Robot Battery health
std_msgs/Header header

# Battery Health
float32 voltage        # Voltage in volts (V)
float32 shunt_voltage  # Shunt voltage in volts (V)
float32 current        # Current in amperes (A)
float32 temperature    # Temperature in degrees Celsius (°C)
float32 power          # Power in watts (W)
float32 energy         # Energy in watt-hours (Wh)
float32 charge_use     # Charge usage in ampere-hours (Ah)