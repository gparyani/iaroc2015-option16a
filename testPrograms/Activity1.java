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
	static EV3MediumRegulatedMotor steeringMotor;
	static NXTUltrasonicSensor rightSensor;
	static NXTUltrasonicSensor leftSensor;
	static EV3IRSensor irSensor;
	static int steeringRange;
	static final int MULTIPLIER = 2;
	static int trueMultiplier = MULTIPLIER;

	public enum Status{
		Forward, Backward, Turning_Left, Turning_Right;
	}
	
	private static Status currentStatus;
	static int ANGLE_ERROR_MARGIN = 5;
	static float[] distances = new float[1];
	static float[] rSamples = new float[1];
	static float[] lSamples = new float[1];
	static float[] gyroAngles = new float[1];
	static float endAngle = 0;
	static float spaceDist;
	static EV3GyroSensor gyro;
	static long beginningTime;
	static SampleProvider rightSense;
	static SampleProvider leftSense;
	static SampleProvider angleSense;
	static SensorMode temp;
	
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
				steeringMotor.rotateTo(0);
				rightMotor.backward();
				leftMotor.backward();
				trueMultiplier = -MULTIPLIER;
				break;
			case Forward:
				steeringMotor.rotateTo(0);
				rightMotor.forward();
				leftMotor.forward();
				trueMultiplier = MULTIPLIER;
				break;
			case Turning_Left:
				steeringMotor.rotateTo(-(steeringRange/2));
				rightMotor.forward();
				leftMotor.forward();
				break;
			case Turning_Right:
				steeringMotor.rotateTo(steeringRange/2);
				rightMotor.forward();
				leftMotor.forward();
			default:
				break;
			}
		}
		currentStatus = newStatus;
	}
	
	public static void alignSteering()
	{
		steeringMotor.forward();
		
		while(!steeringMotor.isStalled());
		
		steeringMotor.stop();
		Delay.msDelay(250);
		int rCount = steeringMotor.getTachoCount();
		System.out.println(rCount);
		
		steeringMotor.backward();
		
		while(!steeringMotor.isStalled());
		
		steeringMotor.stop();
		Delay.msDelay(250);
		int lCount = steeringMotor.getTachoCount();
		System.out.println(lCount);
		
		steeringRange = Math.abs(rCount-lCount);
		int average = (lCount + rCount)/2;
		System.out.println(average);
		
		steeringMotor.rotateTo(average);
		steeringMotor.resetTachoCount();
	
	}
	
	public static void main(String[] args) {
		setExitMode();
		
		initializeMotors();
		
		initializeSensors();
		
		alignSteering();
		
		calibrateGyro();
		
		setStatus(Status.Forward);
		
		beginningTime = System.nanoTime();
		
//Loop Function
		for(int i = 1; true; ++i)
		{
			
//Update Sensors
			
			System.out.println("Right sense: "+rSamples[0]+" Left sense: " + lSamples[0]);
//			tachoReading = leftMotor.getTachoCount(); //TachoCount will get lower when moving backwards
//			System.out.println("After getting values at iteration " + i + ":\t" + (System.nanoTime() - beginningTime));
//			Left Following Strategy
			switch(getStatus()){
			case Backward:
				rightSense.fetchSample(rSamples, 0);
				leftSense.fetchSample(lSamples, 0);
				angleSense.fetchSample(gyroAngles, 0);
				if(gyroAngles[0] < -1 * ANGLE_ERROR_MARGIN || gyroAngles[0] > ANGLE_ERROR_MARGIN)	//too much to the right
					steeringMotor.rotateTo((int) (trueMultiplier * gyroAngles[0]));
				else
					steeringMotor.rotateTo(0);
				System.out.println("After correcting veer at iteration " + i + ":\t" + (System.nanoTime() - beginningTime));
//				tachoReading = leftMotor.getTachoCount();
				if(rSamples[0]>spaceDist && rSamples[0] != Float.POSITIVE_INFINITY)
				{
					setStatus(Status.Turning_Right);
					System.out.println("Turning Right " + i);
					angleSense.fetchSample(gyroAngles, 0);
					endAngle = gyroAngles[0]-90;
					
				}
				break;
			case Forward:
				rightSense.fetchSample(rSamples, 0);
				leftSense.fetchSample(lSamples, 0);
				temp.fetchSample(distances, 0);
				System.out.println("Distance:\t" + distances[0]);
				angleSense.fetchSample(gyroAngles, 0);
				if(gyroAngles[0] < -1 * ANGLE_ERROR_MARGIN || gyroAngles[0] > ANGLE_ERROR_MARGIN)	//too much to the right
					steeringMotor.rotateTo((int) (trueMultiplier * gyroAngles[0]));
				else
					steeringMotor.rotateTo(0);
//				System.out.println("After correcting veer at iteration " + i + ":\t" + (System.nanoTime() - beginningTime));
				if(distances[0]<25)
				{
					System.out.println("Moving Backward " + i);
					setStatus(Status.Backward);
				}
				else if(lSamples[0]>spaceDist && lSamples[0] != Float.POSITIVE_INFINITY)
				{
					setStatus(Status.Turning_Left);
					System.out.println("Turning Left " + i);
					angleSense.fetchSample(gyroAngles, 0);
					endAngle = gyroAngles[0]+90;
				}
				break;
			case Turning_Left:
				angleSense.fetchSample(gyroAngles, 0);
				System.out.println("Current Angle: " + gyroAngles[0]);
				System.out.println("Desired Angle: " + endAngle);
				if(gyroAngles[0]>endAngle)
				{
					setStatus(Status.Forward);
					System.out.println("Moving Forward " + i);
				}
				break;
			case Turning_Right:
				angleSense.fetchSample(gyroAngles, 0);
				System.out.println("Current Angle: " + gyroAngles[0]);
				System.out.println("Desired Angle: " + endAngle);
				if(gyroAngles[0]<endAngle)
				{
					setStatus(Status.Forward);
					System.out.println("Moving Forward " + i);
				}
				break;
			default:
				break;
			}
		}
	}

	private static void setExitMode() {
		new Thread(new Runnable() {
			public void run()
			{
				Button.waitForAnyPress();
				System.exit(0);
			}
		}).start();
	}

	private static void calibrateGyro() {
		System.out.println("Calibrating gyro...ROBOT MUST BE MOTIONLESS");
		gyro = new EV3GyroSensor(SensorPort.S1);
		Delay.msDelay(2000);
		System.out.println("Calibration complete!");
		angleSense = gyro.getAngleMode();
	}

	private static void initializeSensors() {
		rightSensor = new NXTUltrasonicSensor(SensorPort.S3);
		leftSensor = new NXTUltrasonicSensor(SensorPort.S2);
		irSensor = new EV3IRSensor(SensorPort.S4);
		//Adapts to all size boxes
		leftSensor.fetchSample(lSamples, 0);
		float leftDist = lSamples[0];
		rightSensor.fetchSample(rSamples, 0);
		float rightDist = rSamples[0];
		spaceDist = leftDist + rightDist;
		System.out.println("Left Space: " + leftDist);
		System.out.println("Right Space: " + rightDist);
		System.out.println("Total Space:" + spaceDist);
		temp = irSensor.getDistanceMode();
		rightSensor.enable();
		leftSensor.enable();
		rightSense = rightSensor.getDistanceMode();
		leftSense = leftSensor.getDistanceMode();
	}

	private static void initializeMotors() {
		leftMotor = new EV3LargeRegulatedMotor(MotorPort.B);
		rightMotor = new EV3LargeRegulatedMotor(MotorPort.C);
		steeringMotor = new EV3MediumRegulatedMotor(MotorPort.A);
		leftMotor.setSpeed(120);
		rightMotor.setSpeed(120);
		steeringMotor.setSpeed(steeringMotor.getMaxSpeed());
	}
}
