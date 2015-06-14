package testPrograms;

import java.util.Arrays;

import lejos.hardware.Button;
import lejos.hardware.port.I2CPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.I2CSensor;

public class NewClass {

	public static void main(String[] args)
	{
		I2CPort port = new I2CSensor(SensorPort.S2).getPort();
		byte[] toWrite = {1}, readout = new byte[92];
		int address = 0x10;
		while(Button.ESCAPE.isUp())
		{
			port.i2cTransaction(address, toWrite, 0, 1, readout, 0, 92);
			byte[] manufacturerBytes = Arrays.copyOfRange(readout, 0x08, 0x0f);
			String manufacturer = new String(manufacturerBytes);
			System.out.println(manufacturer);
			byte[] a1read = Arrays.copyOfRange(readout, 0x44, 0x45);
			System.out.println(a1read[0] << 2 + a1read[1]);
		}
	}
}
