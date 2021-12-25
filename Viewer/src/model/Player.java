package model;

import utilities.FinalSpeedEnum;
import utilities.GoalState;
import utilities.Position;

public class Player {

	private final static double intervalPeriodMilliseconds = 100.0;
	private Position currentPosition = new Position(0, 0);
	private GoalState goalState;
	private boolean goalPositionChanged = false;
	private double maximumVelocity; // m/s
	private double runningSpeed; // m/s
	private double maxAcceleration; // m/s/s
	private double maxDeceleration; // m/s/s
	private double velocityX; // m/s
	private double velocityY; // m/s
	private Position previousPosition = new Position(0, 0);
	private FinalSpeedEnum initialSpeedEnum = FinalSpeedEnum.Stationary;

	// for knowing current position
	double t;
	double ax;
	double ay;
	double vix;
	double viy;

	public Player(Position initialPosition, double runningSpeed, double maximumVelocity, double acceleration,
			double deceleration) {
		this.currentPosition = initialPosition;
		this.maximumVelocity = maximumVelocity;
		this.runningSpeed = runningSpeed;
		this.maxAcceleration = acceleration;
		this.maxDeceleration = deceleration;
	}

	public void update() {
		t += 0.1;
		updateVelocity();
		if (goalPositionChanged) {
			goalPositionChanged = false;
			t = 0;
		}
		double vfx;
		double vfy;
		vix = velocityX;
		viy = velocityY;
		double x = goalState.getPosition().getX() - currentPosition.getX();
		double y = goalState.getPosition().getY() - currentPosition.getY();
		// kinematic equations for getting to goal asap
		if (goalState.getFinalSpeedEnum() == FinalSpeedEnum.Stationary) {
			// not moving at goal, vf=0
			vfx = 0;
			vfy = 0;
			if (initialSpeedEnum == FinalSpeedEnum.Running) {
				// currently running, use deceleration to stop
				double d = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
				double vi = Math.sqrt(Math.pow(vix, 2) + Math.pow(viy, 2));
				double vf = Math.sqrt(Math.pow(vfx, 2) + Math.pow(vfy, 2));
				double t = goalState.getDesiredTime() == -1 ? (2 * d) / (vf + vi) : goalState.getDesiredTime();
				// magnitude equations are done, break up into x and y
				ax = (x - vix * t) * 2.0 / (Math.pow(t, 2));
				ay = (y - viy * t) * 2.0 / (Math.pow(t, 2));
			}
		} else { // running at final point
			vix = 0;
			viy = 0;
			double a = maxAcceleration;
			if (initialSpeedEnum == FinalSpeedEnum.Stationary) {
				vix = 0;
				viy = 0;
				double d = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
				double t = goalState.getDesiredTime() == -1 ? Math.sqrt(d * 2.0 / a) : goalState.getDesiredTime();
				vfx = x * 2.0 / t - vix;
				vfy = y * 2.0 / t - viy;
				// v^2=vi^2 + 2*a*x
				// x=v_i*t+0.5*a*t^2 -> (x-v_ix*t)/(0.5*t^2)
				ax = (x - vix * t) / (0.5 * Math.pow(t, 2));
				ay = (y - viy * t) / (0.5 * Math.pow(t, 2));
				a = Math.sqrt(Math.pow(ax, 2) + Math.pow(ay, 2));
				if (a > maxAcceleration) {
					// account for maximum acceleration
					ax = ax * maxAcceleration / a;
					ay = ay * maxAcceleration / a;
				}
			}
		}
		// account for hitting max velocity
		//
		double vx = vix + ax * t;
		double vy = viy + ay * t;
		double dxBeforeMaxSpeed = 0;
		double dyBeforeMaxSpeed = 0;
		double dxConstantSpeed = 0;
		double dyConstantSpeed = 0;
		double v = Math.sqrt(Math.pow(vx, 2) + Math.pow(vy, 2));
		if (v > maximumVelocity) {
			double tBeforeMaxSpeed = (maximumVelocity - Math.sqrt(Math.pow(vix, 2) + Math.pow(viy, 2)))
					/ maxAcceleration;
			vx = vx * maximumVelocity / v;
			vy = vy * maximumVelocity / v;
			// v^2 = v0^2+2ax
			dxBeforeMaxSpeed = (Math.pow(vx, 2) - Math.pow(vix, 2)) / 2.0 / ax;
			dyBeforeMaxSpeed = (Math.pow(vy, 2) - Math.pow(viy, 2)) / 2.0 / ay;
			dxConstantSpeed = vx * (t - tBeforeMaxSpeed);
			dyConstantSpeed = vy * (t - tBeforeMaxSpeed);
			ax = 0;
			ay = 0;

		}
		currentPosition = new Position(dxBeforeMaxSpeed + vix * t + 0.5 * ax * Math.pow(t, 2) + dxConstantSpeed,
				dyBeforeMaxSpeed + viy * t + 0.5 * ay * Math.pow(t, 2) + dyConstantSpeed);
	}

	private void updateVelocity() {
		// look at last two data points to determine current speed
		velocityX = (currentPosition.getX() - previousPosition.getX()) * intervalPeriodMilliseconds / 1000.0;
		velocityY = (currentPosition.getY() - previousPosition.getY()) * intervalPeriodMilliseconds / 1000.0;

		// account for maximum speed
		double velocity = Math.sqrt(Math.pow(velocityX, 2) + Math.pow(velocityY, 2));
		if (velocity > maximumVelocity) {
			velocityX = velocityX * maximumVelocity / velocity;
			velocityY = velocityY * maximumVelocity / velocity;
		}
		previousPosition = currentPosition;

	}

	public Position getCurrentPosition() {
		return currentPosition;
	}

	public void setGoalState(GoalState goalState) {
		this.goalState = goalState;
		goalPositionChanged = true;
		update();
	}

}
