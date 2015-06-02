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
		boolean buttonPress = false;
		float[] distances = new float[1];
		SensorMode temp = irSensor.getDistanceMode();
		//TODO: Check if Button.waitForAnyPress() is used correctly
		while(!buttonPress)
		{
			temp.fetchSample(distances, 0);
			System.out.println("Distance to obstacle: "+distances[0]);
			if(distances[0]>10.0)
			{
				leftMotor.forward();
				rightMotor.forward();
			}
			else
			{
				leftMotor.backward();
				rightMotor.backward();
			}
			switch(Button.waitForAnyPress()){
			case Button.ID_ALL:
				buttonPress = true;
				break;
			case Button.ID_DOWN:
				buttonPress = true;
				break;
			case Button.ID_ENTER:
				buttonPress = true;
				break;
			case Button.ID_ESCAPE:
				buttonPress = true;
				break;
			case Button.ID_LEFT:
				buttonPress = true;
				break;
			case Button.ID_RIGHT:
				buttonPress = true;
				break;
			case Button.ID_UP:
				buttonPress = true;
				break;
			}
		}
	}

}
