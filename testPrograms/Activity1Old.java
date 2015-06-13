package testPrograms;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Activity1Old {

	public static void main(String[] args) {
		EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.B);
		EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.C);
		EV3IRSensor irSensor = new EV3IRSensor(SensorPort.S4);
		float[] distances = new float[1];
		SensorMode temp = irSensor.getDistanceMode();
		//TODO: Check if Button.waitForAnyPress() is used correctly
		
		System.out.println("WARNING: Robot must be motionless now for gyro to work");
		System.out.println("Robot will start in 6 seconds");
		Delay.msDelay(4500);	//avoid having to handle InterruptedException with Thread.sleep
		
		new Thread(new GyroSteeringThread()).start();
		
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
		}
		
		leftMotor.stop();
		rightMotor.stop();
		
	
	}

}

class GyroSteeringThreadOld implements Runnable
{
	@Override
	public void run() {
		//Robot MUST BE MOTIONLESS when below line is executed
		try (EV3GyroSensor sensor = new EV3GyroSensor(SensorPort.S1); 
				EV3MediumRegulatedMotor steeringMotor = new EV3MediumRegulatedMotor(MotorPort.A))	//avoid resource leak warning
		{
			Delay.msDelay(1500);	//reset delay
			SampleProvider provider = sensor.getAngleMode();
			steeringMotor.resetTachoCount();
			float[] value = new float[1];
			final int ANGLE_ERROR_MARGIN = 5, MULTIPLIER = 2;
			while(true)
			{
				provider.fetchSample(value, 0);
				if(value[0] < -1 * ANGLE_ERROR_MARGIN && value[0] > ANGLE_ERROR_MARGIN)	//too much to the right
					steeringMotor.rotateTo((int) (MULTIPLIER * value[0]));
				else
					steeringMotor.rotateTo(0);
			}
		}
	}
	
}
