package helperClasses;

import lejos.hardware.port.I2CPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.I2CSensor;
import lejos.robotics.SampleProvider;

/**
 * Obtains values from Ping))) ultrasonic distance sensors connected to the Arduino. Uses I<sup>2</sup>C.
 * 
 * @author Gaurav Paryani
 *
 */
public class ArduinoSideSensors {

	private I2CPort port;
	private int address;
	private int leftValue;
	private int rightValue;
	
	/**
	 * Creates a new instance of this class for one Arduino.
	 * @param port the port to which the Arduino is attached
	 * @param address the 7-bit I<sup>2</sup>C address of the Arduino
	 */
	public ArduinoSideSensors(Port port, int address){
		this.port = (new I2CSensor(port)).getPort();
		this.address = address << 1;	//convert 7-bit address to 8-bit
	}
	
	byte[] values = new byte[8],
			emptyArray = {1};	
	
	/**
	 * Private helper method for reading the values from the Arduino.
	 */
	private void getValues() {
		port.i2cTransaction(address, emptyArray, 0, 1, values, 0, 8);
//		System.out.println(java.util.Arrays.toString(values));
		leftValue = toUnsignedInt(values[1]);	//values are returned from Arduino as unsigned values
		rightValue = toUnsignedInt(values[0]);
	}
	
	/**
	 * Helper method to convert signed bytes into unsigned integers. The values are returned from the Arduino as unsigned, but Java treats them as signed.
	 * @param x the signed byte
	 * @return the unsigned representation of the byte
	 */
	private static int toUnsignedInt(byte x)
    {
	    return ((int) x) & 0xff;
    }
	
	/**
	 * Returns a {@link SampleProvider} that obtains values from the both side sensors.
	 * @return the {@link SampleProvider} for the both sensors
	 */
	public SampleProvider getBothSensorMode()
	{
		return new SampleProvider()
		{
			@Override
			public int sampleSize() {
				return 2;
			}

			@Override
			public void fetchSample(float[] sample, int offset) {
				getValues();
				sample[offset] = leftValue * 0.01f;
				sample[offset+1] = rightValue * 0.01f;
			}
			
		};
	}
	
	/**
	 * Returns a {@link SampleProvider} that obtains values from the right side sensor.
	 * @return the {@link SampleProvider} for the right sensor
	 */
	public SampleProvider getRightSensorMode()
	{
		return new SampleProvider()
		{
			@Override
			public int sampleSize() {
				return 1;
			}

			@Override
			public void fetchSample(float[] sample, int offset) {
				getValues();
				sample[offset] = rightValue * 0.01f;
			}
			
		};
	}
}
