package testPrograms;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
public class Activity2 {

	public static void main(String[] args) {

		EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor (MotorPort.B);
		EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor (MotorPort.C);
		leftMotor.setSpeed(90);
		rightMotor.setSpeed(90);
		leftMotor.forward();
		rightMotor.forward();
		
		while(true)
		{
			int reading = leftMotor.getTachoCount ();
			System.out.println(reading);
			if(reading % 412 == 0)
				Sound.beep();
		}
		
	}

}
