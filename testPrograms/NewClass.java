package testPrograms;

import ch.aplu.ev3.LegoRobot;
import ch.aplu.ev3.SuperProSensor;

public class NewClass {

	public static void main(String[] args)
	{
		LegoRobot robot = new LegoRobot();
		SuperProSensor sensor = new SuperProSensor(ch.aplu.ev3.SensorPort.S2);
		robot.addPart(sensor);
		
		System.out.println(sensor.getVersion());
		
		int[] readout = new int[4];
		
		while(true)
		{
			sensor.readAnalog(readout);
			System.out.println(readout[1]);
		}
	}
}
