package testPrograms;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.SensorMode;

public class Activity1 {

	public static void main(String[] args) {
		EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.B);
		EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.C);
		EV3IRSensor irSensor = new EV3IRSensor(SensorPort.S4);
		float[] distances = new float[1];
		SensorMode temp = irSensor.getDistanceMode();
		//TODO: Check if Button.waitForAnyPress() is used correctly
		
		new Thread(new Runnable() {
			public void run()
			{
				Button.waitForAnyPress();
				System.exit(0);
			}
		}).start();
		
		leftMotor.setSpeed(90);
		rightMotor.setSpeed(90);
		leftMotor.forward();
		rightMotor.forward();
		
		do
		{
			temp.fetchSample(distances, 0);
		}
		while(distances[0] > 10);
		
		leftMotor.backward();
		rightMotor.backward();
	}

}
