# mlx90620

Infrared sensor driver. The inherited calibration and measurement register map follows the MLX90621 datasheet. MLX90620 hardware compatibility has not been established.

Check sensor.check after a measurement or calibration operation. readConfigChecked reports configuration read failures. Invalid temperature calculations produce NaN.

Run make to initialize the pinned soft_i2c submodule if needed and build build/libmlx90620.a. Run make test for host-side tests with a simulated I²C bus.

For Raspberry Pi applications, build soft_i2c from the submodule with WiringPi installed. Link libmlx90620.a before libsoft_i2c.a, followed by -lwiringPi and -lm. Initialize the default bus with setupI2C before calling the driver.
