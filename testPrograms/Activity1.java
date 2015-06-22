package testPrograms;

import lejos.hardware.Button;
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
	static final int MULTIPLIER = 4;
	static int trueMultiplier = MULTIPLIER;

	public enum Status{
		Forward, Backward, Turning_Left, Turning_Right;
	}
	
	private static Status currentStatus;
	static int ANGLE_ERROR_MARGIN = 5;
	static float[] frontSamples = new float[1];
	static float[] rSamples = new float[1];
	static float[] lSamples = new float[1];
	static float[] gyroAngles = new float[1];
	static int endAngle = 0;
	static float spaceDist;
	static volatile double realX = 0;
	static volatile double realY = 0;
	static int lPrevTacho = 0;
	static int rPrevTacho = 0;
	static EV3GyroSensor gyro;
	static long beginningTime;
	static SampleProvider rightSense;
	static SampleProvider leftSense;
	static SampleProvider angleSense;
	static SensorMode frontSense;
	static int steerPos = 0;
	
	public static Status getStatus()
	{
		return currentStatus;
	}
	
	public static void setStatus(Status newStatus)
	{
		if( newStatus != currentStatus)
		{
			System.out.println("State transition: " + newStatus);
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
				steeringMotor.rotateTo(steeringRange / 3);
				rightMotor.rotate(-90, true);
				leftMotor.rotate(-90);
				steeringMotor.rotateTo(-(steeringRange/3));
				rightMotor.forward();
				leftMotor.forward();
				endAngle = correctAngle(gyroAngles[0])+90;
				break;
			case Turning_Right:
				steeringMotor.rotateTo(-steeringRange / 3);
				rightMotor.rotate(-90, true);
				leftMotor.rotate(-90);
				steeringMotor.rotateTo(steeringRange/3);
				rightMotor.forward();
				leftMotor.forward();
				endAngle = correctAngle(gyroAngles[0])-90;
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
		
		startLocationMode();
		
		beginningTime = System.nanoTime();
		
//Loop Function
		for(int i = 1; true; ++i)
		{
			
//Update Sensors
			
//			tachoReading = leftMotor.getTachoCount(); //TachoCount will get lower when moving backwards
//			System.out.println("After getting values at iteration " + i + ":\t" + (System.nanoTime() - beginningTime));
//			Left Following Strategy
			switch(getStatus()){
			case Backward:
				rightSense.fetchSample(rSamples, 0);
				leftSense.fetchSample(lSamples, 0);
//				angleSense.fetchSample(gyroAngles, 0);
//				System.out.println("Current Gyro Angle: " + gyroAngles[0]);
				correctVeer();
//				System.out.println("After correcting veer at iteration " + i + ":\t" + (System.nanoTime() - beginningTime));
//				tachoReading = leftMotor.getTachoCount();
				if(rSamples[0]>spaceDist)
				{
					System.out.println("Right sense: "+rSamples[0]+" Left sense: " + lSamples[0]);
					setStatus(Status.Turning_Right);
//					System.out.println("End Angle: " + endAngle);
					angleSense.fetchSample(gyroAngles, 0);
					
				}
				else if(leftMotor.isStalled() && rightMotor.isStalled())
				{
					setStatus(Status.Forward);
				}
				break;
			case Forward:
				rightSense.fetchSample(rSamples, 0);
				leftSense.fetchSample(lSamples, 0);
				correctVeer();
				frontSense.fetchSample(frontSamples, 0);
//				System.out.println("Distance:\t" + frontSamples[0]);
//				System.out.println("After correcting veer at iteration " + i + ":\t" + (System.nanoTime() - beginningTime));
				if(lSamples[0]>spaceDist)
				{
					System.out.println("Right sense: "+rSamples[0]+" Left sense: " + lSamples[0]);
					setStatus(Status.Turning_Left);
					System.out.println("End Angle: " + endAngle);
					angleSense.fetchSample(gyroAngles, 0);
				}
				else if(frontSamples[0]<30)
				{
//					System.out.println("Moving Backward " + i);
					setStatus(Status.Backward);
				}
				break;
			case Turning_Left:
				angleSense.fetchSample(gyroAngles, 0);
//				System.out.println("Current Angle: " + gyroAngles[0]);
//				System.out.println("Desired Angle: " + endAngle);
				if(gyroAngles[0] >= (endAngle - (4*ANGLE_ERROR_MARGIN)))
				{
					setStatus(Status.Forward);
//					System.out.println("Moving Forward " + i);
				}
				break;
			case Turning_Right:
				angleSense.fetchSample(gyroAngles, 0);
//				System.out.println("Current Angle: " + gyroAngles[0]);
//				System.out.println("Desired Angle: " + endAngle);
				if(gyroAngles[0] <= (endAngle + (4 *ANGLE_ERROR_MARGIN)))
				{
					setStatus(Status.Forward);
//					System.out.println("Moving Forward " + i);
				}
				break;
			default:
				break;
			}
		}
		
	}

	private static void correctVeer() {
		int newSteerPos;
		if( lSamples[0] < 0.1f )
		{
			newSteerPos = -10;
		}
		else if( rSamples[0] < 0.1f )
		{
			newSteerPos = 10;
		}
		else
		{
			angleSense.fetchSample(gyroAngles, 0);
			if(gyroAngles[0] < (endAngle - ANGLE_ERROR_MARGIN) || gyroAngles[0] > (endAngle + ANGLE_ERROR_MARGIN))
				newSteerPos = (int) (trueMultiplier * (gyroAngles[0] - endAngle));
			else
				newSteerPos = 0;
		}
		
		System.out.println("Steering to " + steerPos);
		
		if( steerPos != newSteerPos )
		{
			steeringMotor.rotateTo( newSteerPos );
			steerPos = newSteerPos;
		}
		
	}

	private static void setExitMode() {
		new Thread(new Runnable() {
			public void run()
			{
				while(Button.ESCAPE.isUp());
				System.exit(0);
			}
		}).start();
	}

	private static void calibrateGyro() {
		System.out.println("Calibrating gyro...ROBOT MUST BE MOTIONLESS");
		Delay.msDelay(200);
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
		spaceDist = leftDist + rightDist + 0.19f+0.0762f; //19=robot + 0.0762 = wall width
		System.out.println("Left Space: " + leftDist);
		System.out.println("Right Space: " + rightDist);
		System.out.println("Total Space:" + spaceDist);
		frontSense = irSensor.getDistanceMode();
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
	private static void updateLocation() {
		int lTachoDelta, rTachoDelta;
		double avgDelta;
		double xDelta, yDelta;
		double gyroRadians;
		lTachoDelta = leftMotor.getTachoCount() - lPrevTacho;
		rTachoDelta = rightMotor.getTachoCount() - rPrevTacho;
		lPrevTacho += lTachoDelta;
		rPrevTacho += rTachoDelta;
		angleSense.fetchSample(gyroAngles, 0);
		gyroRadians = Math.toRadians(gyroAngles[0] + 90);
		avgDelta = (lTachoDelta+rTachoDelta)/2.0;
		xDelta = (avgDelta) * (1.0/412.0) * Math.cos(gyroRadians);
		yDelta = (avgDelta) * (1.0/412.0) * Math.sin(gyroRadians);
		realX += xDelta;
		realY += yDelta;
	}
	private static void startLocationMode() {
		Thread t = new Thread(new Runnable()  {
			public void run()
			{
				while( true )
				{
					updateLocation();
//					System.out.println(getStatus().toString()+", x: "+realX + ", y: " +realY + " left: " + lSamples[0] + " right: " + rSamples[0] );
				}
			}
		});
//		t.setPriority(Thread.MAX_PRIORITY);
		t.start();
	}
	
	private static int correctAngle(float angleEstimate)
	{
		float quotient;
		float remainder;
		quotient = angleEstimate/90;
		remainder = angleEstimate%90;
		if(remainder>=0)
		{
			if(remainder>=45)
			{
				quotient+=1;
			}	
		}
		else
		{
			if(remainder<=-45)
			{
				quotient-=1;
			}
		}
		return (int)quotient*90;
	}
}
