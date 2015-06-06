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
		
		while(true)
		{
			temp.fetchSample(distances, 0);
			System.out.println("Distance to obstacle: "+distances[0]);
			if(distances[0]>10.0) //10 cm
			{
				
				leftMotor.forward();
				rightMotor.forward();
			}
			else
			{
				leftMotor.backward();
				rightMotor.backward();
			}
		}
	}

}
