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
//Declarations
		EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.B);
		EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.C);
		NXTUltrasonicSensor rightSensor = new NXTUltrasonicSensor(SensorPort.S3);
		NXTUltrasonicSensor leftSensor = new NXTUltrasonicSensor(SensorPort.S2);
		EV3IRSensor irSensor = new EV3IRSensor(SensorPort.S4);
		RobotStatus status = RobotStatus.Forward;
		int tachoReading;
		float[] distances = new float[1];
		float[] rSample = new float[1];
		float[] lSample = new float[1];
		
//Set-up
		SensorMode temp = irSensor.getDistanceMode();
		rightSensor.enable();
		leftSensor.enable();
		SampleProvider rightSense = rightSensor.getDistanceMode();
		SampleProvider leftSense = leftSensor.getDistanceMode();
		leftMotor.setSpeed(90);
		rightMotor.setSpeed(90);
		new Thread(new Runnable() {
			public void run()
			{
				Button.waitForAnyPress();
				System.exit(0);
			}
		}).start();
		
//Loop Function
		while(true)
		{
			
//Update Sensors
			temp.fetchSample(distances, 0);
			rightSense.fetchSample(rSample, 0);
			leftSense.fetchSample(lSample, 0);
			tachoReading = leftMotor.getTachoCount(); //TachoCount will get lower when moving backwards
			if(status == RobotStatus.Forward)
			{
				leftMotor.forward();
				rightMotor.forward();
				if(distances[0]<10)
				{
					leftMotor.stop();
					rightMotor.stop();
					status = RobotStatus.Backward;
				}
			}
			else
			{
				leftMotor.backward();
				rightMotor.backward();
				tachoReading = leftMotor.getTachoCount();
				if(tachoReading == 0)
				{
					leftMotor.stop();
					rightMotor.stop();
					status = RobotStatus.Forward;
				}
			}
			if(rSample[0]>1.0)
			{
				Sound.twoBeeps();
			}
			if(lSample[0]>1.0)
			{
				Sound.buzz();
			}
		}
	}
	
	public enum RobotStatus{
		Forward, Backward;
	}
}
