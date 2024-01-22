# PSoC adxl345 tripple access Accelerometer
This is an attempt to get a SparkFun adxl345 working on a PSoC 5LP Master SPI controller https://www.sparkfun.com/products/9836.

This code was converted to C from the Arduino Sample code from https://github.com/sparkfun/ADXL345_Breakout repo.

Originally I had attempted to use an inexpensive logic level converter and a 5V->3.3V buck converter to create a 3.3V power supply.

The logic level converter had far to much RC lag to be useful at rated frequencies.

To improve signal integrity, the 0 Ohm resistor R15 was removed to allow VDDIO to be directly powered by 3.3V while programming with the 5V USB.

Currently this code works but I am seeing periodic errors in accelerometer values that appear to be from missaligned bits possibly from some timing slack issues.
