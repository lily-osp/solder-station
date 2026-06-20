# Compatible Soldering Irons for Custom Controller

Based on the specifications of our custom soldering iron controller, the following soldering irons should be compatible and work well out of the box:

| Soldering Iron Model | Voltage | Power | Temp Range | Compatible Tips | Key Features | Compatibility Rationale |
| :--- | :---: | :---: | :---: | :--- | :--- | :--- |
| **Hakko FX-8801** | 24V | 65W | 200°C - 480°C | T18 Series | Ceramic heating element | Matches 24V supply and target temperature range. |
| **KSGER T12 Handle** | 24V | 72W | N/A | T12 Series | Direct heating cartridge | Ideal for DIY; easily controllable via 24V PWM. |
| **JBC C245** | 24V | 50W | 90°C - 450°C | C245 Series | Cartridge design tips | Standard 24V operation within control range. |
| **Weller WSP 80** | 24V | 80W | 50°C - 450°C | LT / WS80 Series | Heavy-duty element | Operates at 24V with excellent thermal capacity. |
| **Quick 957D** | 24V | 60W | N/A | 900M Series | Accessible replacements | Designed for 24V, matches PWM gate driving. |

## Important Selection Criteria

When selecting a soldering iron, consider the following:

- **Voltage:** Ensure the iron operates at 24V to match the system power supply.
- **Current Limit:** Verify power consumption doesn't exceed the IRLZ44N MOSFET design limits.
- **Sensor Type:** Check if the temperature sensor (usually a thermocouple) is compatible with the LM358P-based operational amplifier circuit.
- **Tip Economy:** Consider the local availability and cost of replacement tips.

> [!WARNING]
> Always refer to the manufacturer's official specifications before connection. Minor software mapping adjustments or amplification gain settings in the controller code may be required for specific sensor types.
