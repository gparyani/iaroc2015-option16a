package testPrograms;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.NXTUltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Activity1 {

	static EV3LargeRegulatedMotor leftMotor;
	static EV3LargeRegulatedMotor rightMotor;
	static NXTUltrasonicSensor rightSensor;
	static NXTUltrasonicSensor leftSensor;
	static EV3IRSensor irSensor;
	static final int MULTIPLIER = 2;
	static int trueMultiplier = MULTIPLIER;

	public enum Status{
		Forward, Backward;
	}
	
	private static Status currentStatus;
	
	public static Status getStatus()
	{
		return currentStatus;
	}
	
	public static void setStatus(Status newStatus)
	{
		if( newStatus != currentStatus)
		{
			leftMotor.stop();
			rightMotor.stop();
			switch(newStatus){
			case Backward:
				rightMotor.backward();
				leftMotor.backward();
				trueMultiplier = -MULTIPLIER;
				break;
			case Forward:
				rightMotor.forward();
				leftMotor.forward();
				trueMultiplier = MULTIPLIER;
				break;
			default:
				break;
			}
		}
		currentStatus = newStatus;
	}
	
	
	public static void main(String[] args) {
//Declarations
		leftMotor = new EV3LargeRegulatedMotor(MotorPort.B);
		rightMotor = new EV3LargeRegulatedMotor(MotorPort.C);
		EV3MediumRegulatedMotor steeringMotor = new EV3MediumRegulatedMotor(MotorPort.A);
		rightSensor = new NXTUltrasonicSensor(SensorPort.S3);
		leftSensor = new NXTUltrasonicSensor(SensorPort.S2);
		irSensor = new EV3IRSensor(SensorPort.S4);
		EV3GyroSensor gyro;
		leftMotor.setSpeed(90);
		rightMotor.setSpeed(90);
		int tachoReading;
		float[] distances = new float[1],
				rSample = new float[1],
				lSample = new float[1],
				gyroAngle = new float[1];
		final int ANGLE_ERROR_MARGIN = 5;
		
//Set-up
		System.out.println("Calibrating gyro...ROBOT MUST BE MOTIONLESS");
		gyro = new EV3GyroSensor(SensorPort.S1);
		Delay.msDelay(2500);
		System.out.println("Calibration complete!");
		setStatus(Status.Forward);
		SensorMode temp = irSensor.getDistanceMode();
		rightSensor.enable();
		leftSensor.enable();
		SampleProvider rightSense = rightSensor.getDistanceMode();
		SampleProvider leftSense = leftSensor.getDistanceMode();
		SampleProvider angleSense = gyro.getAngleMode();
		new Thread(new Runnable() {
			public void run()
			{
				Button.waitForAnyPress();
				System.exit(0);
			}
		}).start();
		
		long beginningTime = System.nanoTime();
		
//Loop Function
		for(int i = 1; true; ++i)
		{
			
//Update Sensors
			
			rightSense.fetchSample(rSample, 0);
			leftSense.fetchSample(lSample, 0);
			tachoReading = leftMotor.getTachoCount(); //TachoCount will get lower when moving backwards
			System.out.println("After getting values at iteration " + i + ":\t" + (System.nanoTime() - beginningTime));
			if(getStatus() == Status.Forward)
			{
				temp.fetchSample(distances, 0);
				System.out.println("Distance:\t" + distances[0]);
				if(distances[0]<20)
				{
					setStatus(Status.Backward);
				}
			}
			else
			{
				tachoReading = leftMotor.getTachoCount();
				if(tachoReading == 0)
				{
					setStatus(Status.Forward);
				}
			}
			System.out.println("After changing states at iteration " + i + ":\t" + (System.nanoTime() - beginningTime));
//			if(rSample[0]>1.0)
//			{
//				Sound.twoBeeps();
//			}
//			if(lSample[0]>1.0)
//			{
//				Sound.buzz();
//			}
//			System.out.println("After beeping at iteration " + i + ":\t" + (System.nanoTime() - beginningTime));
			angleSense.fetchSample(gyroAngle, 0);
			if(gyroAngle[0] < -1 * ANGLE_ERROR_MARGIN || gyroAngle[0] > ANGLE_ERROR_MARGIN)	//too much to the right
				steeringMotor.rotateTo((int) (trueMultiplier * gyroAngle[0]));
			else
				steeringMotor.rotateTo(0);
			System.out.println("After correcting veer at iteration " + i + ":\t" + (System.nanoTime() - beginningTime));
		}
	}
}
