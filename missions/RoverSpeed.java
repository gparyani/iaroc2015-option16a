package missions;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class RoverSpeed {
	
	static EV3GyroSensor sensor;
	static SampleProvider angleSense;
	static EV3MediumRegulatedMotor steeringMotor;
	static EV3LargeRegulatedMotor leftMotor, rightMotor;
	static float[] data = new float[1];
	static float currentReading;
	static final int ANGLE_ERROR_MARGIN = 5;

	public static void main(String[] args) {
		Button.LEDPattern(2);	//button backlight red
		System.out.println("Calibrating gyro...DO NOT TOUCH ROBOT");
		Delay.msDelay(750);
		sensor = new EV3GyroSensor(SensorPort.S1);
		Delay.msDelay(1750);
		angleSense = sensor.getAngleMode();
		
		Button.LEDPattern(3);	//button backlight yellow
		System.out.println("Aligning steering wheels");
		alignSteering();
		
		Button.LEDPattern(1);	//button backlight green
		System.out.println("Press any button to start");
		Button.waitForAnyPress();
		
		forwardRoutine();
	}
	
	private static void forwardRoutine() {
		leftMotor.setSpeed(leftMotor.getMaxSpeed());
		rightMotor.setSpeed(leftMotor.getMaxSpeed());	//make sure each motor 
		leftMotor.forward();
		rightMotor.forward();
		
		Delay.msDelay(100);	//avoid detecting stalls early
		
		while(!leftMotor.isStalled() && !rightMotor.isStalled())
		{
			currentReading = getDataFromGyro();
			int turnAngle = (int) (-currentReading / 1.1);
			
			//Threshold detection
			if(Math.abs(turnAngle) <= ANGLE_ERROR_MARGIN)
				steeringMotor.rotateTo(0, true);
			else
			{
				turnAngle += (int) Math.signum(turnAngle) * 25;
				steeringMotor.rotateTo(turnAngle, true);
			}
		}
	}
	

	private static float getDataFromGyro() {
		angleSense.fetchSample(data, 0);
		return data[0];
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
		
//		steeringRange = Math.abs(rCount-lCount);
		int average = (lCount + rCount)/2;
		System.out.println(average);
		
		steeringMotor.rotateTo(average);
		steeringMotor.resetTachoCount();
	
	}

}
