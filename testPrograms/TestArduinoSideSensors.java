package testPrograms;

import helperClasses.ArduinoSideSensors;
import lejos.hardware.port.SensorPort;
import lejos.robotics.SampleProvider;

public class TestArduinoSideSensors {

	public static void main(String[] args) {
		ArduinoSideSensors sensors = new ArduinoSideSensors(SensorPort.S2, 4);
		SampleProvider left = sensors.getLeftSensorMode(), right = sensors.getRightSensorMode();
		float[] leftValue = new float[1], rightValue = new float[1];;
		while(true)
		{
			left.fetchSample(leftValue, 0);
			right.fetchSample(rightValue, 0);
			System.out.println("Left: " + leftValue[0] + "\tRight: " + rightValue[0]);
		}

	}

}
