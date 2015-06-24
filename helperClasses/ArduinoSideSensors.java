package helperClasses;

import lejos.hardware.port.I2CPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.I2CSensor;
import lejos.robotics.SampleProvider;

public class ArduinoSideSensors {

	private I2CPort port;
	private int address;
	private int leftValue;
	private int rightValue;
	
	public ArduinoSideSensors(Port port, int address){
		this.port = (new I2CSensor(port)).getPort();
		this.address = address << 1;
	}
	
	byte[] values = new byte[8],
			emptyArray = {1};	
	
	private void getValues() {
		port.i2cTransaction(address, emptyArray, 0, 1, values, 0, 8);
//		System.out.println(java.util.Arrays.toString(values));
		leftValue = toUnsignedInt(values[1]);	//values are returned from Arduino as unsigned values
		rightValue = toUnsignedInt(values[0]);
	}
	
	private static int toUnsignedInt(byte x)
    {
	    return ((int) x) & 0xff;
    }
	
	public SampleProvider getLeftSensorMode()
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
				sample[offset] = leftValue * 0.01f;
			}
			
		};
	}
	
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
