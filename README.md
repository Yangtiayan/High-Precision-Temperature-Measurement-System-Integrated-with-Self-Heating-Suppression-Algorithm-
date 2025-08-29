# High-precision RTD Temperature Measurement System Design Based on ADUCM360

1. Developed Intermittent Modes Driver for Cortex-M0 MCU with Dual 24-Bit Delta-Sigma ADC.
2. Finite Element Simulation of Temperature-Fluid-Electric Coupling. 
3. Designed Hardware Circuit and Developed Self-Heating Suppression Algorithm with Intermittent Operation Modes.

##  Code
The "Measurement Driver and Self-Heating Error Suppression Algorithm" folder contains the driver for the ADuCM360, which is designed for 4-wire RTD measurements. Its primary function is to configure the ADC’s input channels, gain (gain = 4), and operating mode (Sinc3 filter and chopping mode) to ensure high-precision voltage measurements at the hardware level. The folder also includes the algorithmic design for suppressing RTD self-heating errors.

The code implements a continuous "measure-compensate-output" loop. First, it measures the RTD's voltage. Next, it converts the voltage to resistance and then converts the resistance to a temperature value (fTRTD). It then calls the implement_algorithm function to compensate for the self-heating error in fTRTD. Finally, the compensated temperature value is sent out via UART.

The code defines several floating-point arrays, such as error_continuous_mode0 and error_discontinuous_mode1. The values in these arrays are measured or simulated self-heating error values from a research paper for specific operating modes and time points. Inside the implement_algorithm function, a simple switch statement selects the correct error array based on the current operating mode, which is controlled by the currrent_continuous and cooling_time variables. The code then uses a counter (count_number) as an index to retrieve the corresponding error value from the array and subtracts it directly from the current measured temperature to complete the compensation. This "lookup table" method is faster and more lightweight than real-time calculation, making it ideal for resource-constrained microcontrollers.
