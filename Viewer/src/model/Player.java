package model;

import utilities.FinalSpeedEnum;
import utilities.Position;

public class Player {
	
	private final static double intervalPeriodMilliseconds = 100.0;
	private Position currentPosition = new Position(0, 0);
	private Position goalPosition;
	private boolean goalPositionChanged = false;
	private double maximumSpeed; //m/s
	private double runningSpeed; //m/s
	private double acceleration; //m/s/s
	private double deceleration; //m/s/s
	private double velocityX; //m/s
	private double velocityY; //m/s
	private Position previousPosition = new Position(0, 0);
	private FinalSpeedEnum initialSpeedEnum = FinalSpeedEnum.Stationary;
	private FinalSpeedEnum finalSpeedEnum = FinalSpeedEnum.Stationary; 
	
	//for knowing current position
	double t;
	double ax;
	double ay;
	double vix;
	double viy;
	
	public Player(Position initialPosition, double runningSpeed, double maximumSpeed, double acceleration, double deceleration){
		this.currentPosition = initialPosition;
		this.maximumSpeed = maximumSpeed;
		this.runningSpeed = runningSpeed;
		this.acceleration = acceleration;
		this.deceleration = deceleration;
	}
	
	public void update() {
		t += 0.1;
		updateVelocity();
		if (goalPositionChanged) {
			goalPositionChanged = false;
			double vfx;
			double vfy;
			vix = velocityX;
			viy = velocityY;
			double x = goalPosition.getX() - currentPosition.getX();
			double y = goalPosition.getY() - currentPosition.getY();
			//kinematic equations for getting to goal asap
			if (finalSpeedEnum == FinalSpeedEnum.Stationary) {
				//not moving at goal, vf=0
				vfx = 0;
				vfy = 0;
				if (initialSpeedEnum == FinalSpeedEnum.Running) {
					//currently running, use deceleration to stop
					double d = Math.sqrt(Math.pow(x,  2) + Math.pow(y, 2));
					double vi = Math.sqrt(Math.pow(vix, 2) + Math.pow(viy, 2));
					double vf = Math.sqrt(Math.pow(vfx,  2) + Math.pow(vfy, 2));
					double t = (2 * d)/(vf + vi);
					//magnitude equations are done, break up into x and y
					ax = (x - vix*t)*2.0/(Math.pow(t, 2));
					ay = (y - viy*t)*2.0/(Math.pow(t, 2));
				}
			} else { //running at final point
				vix = 0;
				viy = 0;
				double a = acceleration;
				if (initialSpeedEnum == FinalSpeedEnum.Stationary) {
					vix = 0;
					viy = 0;
					double d = Math.sqrt(Math.pow(x, 2) + Math.pow(y,  2));
					double t = Math.sqrt(d*2.0/a);
					vfx = x * 2.0 / t - vix;
					vfy = y * 2.0 / t - viy;
					//v^2=vi^2 + 2*a*x
					//x=v_i*t+0.5*a*t^2 -> (x-v_ix*t)/(0.5*t^2)
					ax = (x-vix*t)/(0.5*Math.pow(t, 2));
					ay = (y-viy*t)/(0.5*Math.pow(t, 2));
				}
			}
			t = 0;
		}
		currentPosition = new Position(vix*t+0.5*ax*Math.pow(t, 2), viy*t+0.5*ay*Math.pow(t, 2));
	}
	
	private void updateVelocity() {
		// look at last two data points to determine current speed
		velocityX = (currentPosition.getX() - previousPosition.getX()) * intervalPeriodMilliseconds / 1000.0;
		velocityY = (currentPosition.getY() - previousPosition.getY()) * intervalPeriodMilliseconds / 1000.0;;
		previousPosition = currentPosition;
		
	}

	public Position getCurrentPosition() {
		return currentPosition;
	}
	
	public void setGoalState(Position goalPosition, FinalSpeedEnum finalSpeed) {
		this.goalPosition = goalPosition;
		finalSpeedEnum = finalSpeed;
		goalPositionChanged = true;
		update();
	}
}
