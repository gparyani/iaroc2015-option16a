package testPrograms;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.NXTUltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;

public class Activity1 {

	public static void main(String[] args) {
		EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.B);
		EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.C);
		NXTUltrasonicSensor rightSensor = new NXTUltrasonicSensor(SensorPort.S3);
		EV3IRSensor irSensor = new EV3IRSensor(SensorPort.S4);
		float[] distances = new float[1];
		float[] rSample = new float[1];
		SensorMode temp = irSensor.getDistanceMode();
		rightSensor.enable();
		SampleProvider rightSense = rightSensor.getDistanceMode();
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
			rightSense.fetchSample(rSample, 0);
			if(rSample[0]>1.0)
			{
				Sound.twoBeeps();
			}
		}
		while(distances[0] > 10); //keep on getting distance while distance is greater than 10. 
								  //check if distance is 10 AFTER you get the distance.
								  //exits loop when distance is less than 10.
		leftMotor.backward();
		rightMotor.backward();
		 
		int reading = leftMotor.getTachoCount(); //TachoCount will get lower when moving backwards
		
		while(reading > 0)
		{
			reading = leftMotor.getTachoCount();
			System.out.println(reading);
			rightSense.fetchSample(rSample, 0);
			if(rSample[0]>1.0)
			{
				Sound.twoBeeps();
			}
		}
		
		leftMotor.stop();
		rightMotor.stop();
		
	
	}

}
