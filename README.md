# paxini_driver
Simple driver for panixi tactile sensor

## Dependencies 

This driver requires the following Python packages:

`pip install numpy pyserial pyyaml`


## Usage

1. Make sure your user has permission to access the serial port:
	`sudo chmod 666 /dev/ttyACM0`

2. Edit `paxini.yaml` to match your hardware configuration (port, sensor IDs, etc).

3. Run the driver and test communication using the `test_sensing()` function in `paxini_driver.py`.
	This will initialize the sensors and print out the sensed data and timing information.
