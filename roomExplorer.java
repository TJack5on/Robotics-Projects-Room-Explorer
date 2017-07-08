import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
//import lejos.hardware.ev3.LocalEV3;
//import lejos.hardware.lcd.Font;
//import lejos.hardware.lcd.GraphicsLCD;
//import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
//import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.RegulatedMotor;
//import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.DifferentialPilot;
//import lejos.robotics.subsumption.Arbitrator;
//import lejos.robotics.subsumption.Behavior;
import lejos.utility.PilotProps;


public class roomExplorer {
	static RegulatedMotor leftMotor;
	static RegulatedMotor rightMotor;
	static RegulatedMotor headMotor = Motor.D;
	static EV3TouchSensor leftTouch;
	static EV3TouchSensor rightTouch;
	
	public static void main(String[] args) throws Exception{
		
		PilotProps pp = new PilotProps();
		pp.loadPersistentValues();
		float wheelDiameter = Float.parseFloat(pp.getProperty(PilotProps.KEY_WHEELDIAMETER, "56.0"));
		float trackWidth = Float.parseFloat(pp.getProperty(PilotProps.KEY_TRACKWIDTH, "122.0"));
		
		leftMotor = PilotProps.getMotor(pp.getProperty(PilotProps.KEY_LEFTMOTOR,"B"));
		rightMotor = PilotProps.getMotor(pp.getProperty(PilotProps.KEY_RIGHTMOTOR, "C"));
		DifferentialPilot robot = new DifferentialPilot(wheelDiameter, trackWidth, leftMotor, rightMotor,false);
		EV3UltrasonicSensor headSensor = new EV3UltrasonicSensor(SensorPort.S4);
		Port leftPort = LocalEV3.get().getPort("S2");
		Port rightPort = LocalEV3.get().getPort("S3");
		EV3TouchSensor leftTouch = new EV3TouchSensor(leftPort);
		EV3TouchSensor rightTouch = new EV3TouchSensor(rightPort);
		//EV3TouchSensor leftSensor = new EV3TouchSensor(SensorPort.S2);
		//EV3TouchSensor rightSensor = new EV3TouchSensor(SensorPort.S3);
		
		SensorMode distanceProvider = (SensorMode) headSensor.getDistanceMode();
		SensorMode touchProvider = (SensorMode) leftTouch.getTouchMode();
		SensorMode touchProvider2 = (SensorMode) rightTouch.getTouchMode();
		
		float[] headData = new float[headSensor.sampleSize()];
		float[] leftData = new float[leftTouch.sampleSize()];
		float[] rightData = new float[rightTouch.sampleSize()];
		
		leftMotor.resetTachoCount();
		rightMotor.resetTachoCount();
		leftMotor.rotateTo(0);
		rightMotor.rotateTo(0);
		leftMotor.setSpeed(200);
		rightMotor.setSpeed(200);
		leftMotor.setAcceleration(600);
		rightMotor.setAcceleration(600);	
		headSensor.enable();
		robot.setRotateSpeed(400);
		touchProvider.fetchSample(leftData, 0);
		touchProvider2.fetchSample(rightData, 0);
		distanceProvider.fetchSample(headData, 0);
		
		while(Button.getButtons() != Button.ID_ESCAPE ){
			distanceProvider.fetchSample(headData, 0);
			touchProvider.fetchSample(leftData, 0);
			touchProvider2.fetchSample(rightData, 0);
			//leftMotor.forward();
			//rightMotor.forward();
			
			if(headData[0]<0.3){
				robot.stop();
				headMotor.rotate(90);
				distanceProvider.fetchSample(headData, 0);
				int rightWall = (int) headData[0];
				headMotor.rotate(-180);
				distanceProvider.fetchSample(headData, 0);
				int leftWall = (int) headData[0];
				headMotor.rotate(90);
					if(rightWall>leftWall){
						robot.rotate(-50);
					}
					else{
						robot.rotate(50);
					}
			}
			else if(leftData[0]>0.9||rightData[0]>0.9){
				robot.stop();
				headMotor.rotate(90);
				distanceProvider.fetchSample(headData, 0);
				int rightWall = (int) headData[0];
				headMotor.rotate(-180);
				distanceProvider.fetchSample(headData, 0);
				int leftWall = (int) headData[0];
				headMotor.rotate(90);
					if(rightWall>leftWall){
						robot.rotate(-90);
					}
					else{
						robot.rotate(90);
					}
			}
			
			else{
				leftMotor.forward();
				rightMotor.forward();
			}
			
		}
		
	}

}
