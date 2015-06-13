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

public class Activity4 {

	public static void main(String[] args) {
//Declarations
		EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.B);
		EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.C);
		EV3MediumRegulatedMotor steeringMotor = new EV3MediumRegulatedMotor(MotorPort.A);
		NXTUltrasonicSensor rightSensor = new NXTUltrasonicSensor(SensorPort.S3);
		NXTUltrasonicSensor leftSensor = new NXTUltrasonicSensor(SensorPort.S2);
		EV3IRSensor irSensor = new EV3IRSensor(SensorPort.S4);
		EV3GyroSensor gyro;	//will initialize later
		RobotStatus status = RobotStatus.Forward;
		int tachoReading;
		float[] distances = new float[1],
				rSample = new float[1],
				lSample = new float[1],
				gyroAngle = new float[1];
		final int ANGLE_ERROR_MARGIN = 5,
				MULTIPLIER = 2;
		
//Set-up
		System.out.println("Calibrating gyro...ROBOT MUST BE MOTIONLESS");
		gyro = new EV3GyroSensor(SensorPort.S1);
		Delay.msDelay(2500);
		System.out.println("Calibration complete!");
		SensorMode temp = irSensor.getDistanceMode();
		rightSensor.enable();
		leftSensor.enable();
		SampleProvider rightSense = rightSensor.getDistanceMode();
		SampleProvider leftSense = leftSensor.getDistanceMode();
		SampleProvider angleSense = gyro.getAngleMode();
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
			tachoReading = leftMotor.getTachoCount();	//TachoCount will get lower when moving backwards
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
			
			angleSense.fetchSample(gyroAngle, 0);
			if(gyroAngle[0] < -1 * ANGLE_ERROR_MARGIN || gyroAngle[0] > ANGLE_ERROR_MARGIN)	//too much to the right
				steeringMotor.rotateTo((int) (MULTIPLIER * gyroAngle[0]));
			else
				steeringMotor.rotateTo(0);
		}
	}
	
	public enum RobotStatus{
		Forward, Backward;
	}
}
