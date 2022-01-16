package model;

import utilities.CurrentState;
import utilities.FinalSpeedEnum;
import utilities.GoalState;
import utilities.Position;

class InitialConditions {
	double vix;
	double viy;
	double dix;
	double diy;
	double dx;
	double dy;
	
	double getInitialVelocity() {
		return Math.sqrt(Math.pow(vix, 2) + Math.pow(viy, 2));
	}

	double getTotalDistance() {
		return Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
	}
}

public class Player {

	private final static double intervalPeriodMilliseconds = 100.0;
	private GoalState goalState;
	private CurrentState currentState = new CurrentState();
	private boolean goalPositionChanged = false;
	private double maximumVelocity; // m/s
	private double runningSpeed; // m/s
	private double maxAcceleration; // m/s/s
	private double maxDeceleration; // m/s/s
	private Position previousPosition = new Position(0, 0);
	private FinalSpeedEnum initialSpeedEnum = FinalSpeedEnum.Stationary;

	// for knowing current position
	private InitialConditions initialConditions = new InitialConditions();
	private double t;
	private double ax;
	private double ay;
	
	public Player(Position initialPosition, double runningSpeed, double maximumVelocity, double acceleration,
			double deceleration) {
		this.currentState.setPosition(initialPosition);
		this.maximumVelocity = maximumVelocity;
		this.runningSpeed = runningSpeed;
		this.maxAcceleration = acceleration;
		this.maxDeceleration = deceleration;
	}

	public void update() {
		t += 0.1;
		if (goalPositionChanged) {
			goalPositionChanged = false;
			t = 0;
			initialConditions.vix = currentState.getVelocityX();
			initialConditions.viy = currentState.getVelocityY();
			if (currentState.getVelocity() > 0) {
				initialSpeedEnum = FinalSpeedEnum.Running;
			} else {
				initialSpeedEnum = FinalSpeedEnum.Stationary;
			}
			initialConditions.dix = currentState.getPosition().getX();
			initialConditions.diy = currentState.getPosition().getY();
			initialConditions.dx = goalState.getPosition().getX() - initialConditions.dix;
			initialConditions.dy = goalState.getPosition().getY() - initialConditions.diy;
		}
		// kinematic equations for getting to goal asap
		if (goalState.getFinalSpeedEnum() == FinalSpeedEnum.Stationary) {
			// not moving at goal, vf=0
			if (initialSpeedEnum == FinalSpeedEnum.Running) {
				updateRunningToStationary();

			}
		} else { // running at final point
			initialConditions.vix = 0;
			initialConditions.viy = 0;
			if (initialSpeedEnum == FinalSpeedEnum.Stationary) {
				updateStationaryToRunning();	
			}
		}
	}

	public Position getCurrentPosition() {
		return currentState.getPosition();
	}
	
	public double getVelocityX() {
		return currentState.getVelocityX();
	}
	
	public double getVelocityY() {
		return currentState.getVelocityY();
	}

	public void setGoalState(GoalState goalState) {
		this.goalState = goalState;
		goalPositionChanged = true;
		update();
	}
	
	private void updateStationaryToRunning() {
		double dxBeforeMaxSpeed = 0;
		double dyBeforeMaxSpeed = 0;
		double dxConstantSpeed = 0;
		double dyConstantSpeed = 0;
		double a = maxDeceleration;
		initialConditions.vix = 0;
		initialConditions.viy = 0;
		double timeDesired = goalState.getDesiredTime() == -1 ? Math.sqrt(initialConditions.getTotalDistance() * 2.0 / a) : goalState.getDesiredTime();
		//find acceleration
		// v^2=vi^2 + 2*a*x
		// x=v_i*t+0.5*a*t^2 -> (x-v_ix*t)/(0.5*t^2)
		ax = (initialConditions.dx - initialConditions.vix * timeDesired) / (0.5 * Math.pow(timeDesired, 2));
		ay = (initialConditions.dy - initialConditions.viy * timeDesired) / (0.5 * Math.pow(timeDesired, 2));
		a = Math.sqrt(Math.pow(ax, 2) + Math.pow(ay, 2));
		if (a > maxAcceleration) {
			// account for maximum acceleration
			ax = ax * maxAcceleration / a;
			ay = ay * maxAcceleration / a;
		}
		//find current velocity
		double vx = initialConditions.vix + ax * t;
		double vy = initialConditions.viy + ay * t;
		// account for hitting max velocity
		//
		double v = Math.sqrt(Math.pow(vx, 2) + Math.pow(vy, 2));
		if (v > maximumVelocity) {
			double tBeforeMaxSpeed = (maximumVelocity - initialConditions.getInitialVelocity())
					/ maxAcceleration;
			vx = vx * maximumVelocity / v;
			vy = vy * maximumVelocity / v;
			// v^2 = v0^2+2ax
			if (ax != 0.0)
				dxBeforeMaxSpeed = (Math.pow(vx, 2) - Math.pow(initialConditions.vix, 2)) / 2.0 / ax;
			else
				dxBeforeMaxSpeed = initialConditions.vix * tBeforeMaxSpeed;
			if (ay != 0.0)
				dyBeforeMaxSpeed = (Math.pow(vy, 2) - Math.pow(initialConditions.viy, 2)) / 2.0 / ay;
			else
				dyBeforeMaxSpeed = initialConditions.viy * tBeforeMaxSpeed;
			dxConstantSpeed = vx * (t - tBeforeMaxSpeed);
			dyConstantSpeed = vy * (t - tBeforeMaxSpeed);
			ax = 0;
			ay = 0;
		}
		currentState.setPosition(new Position(initialConditions.dix + dxBeforeMaxSpeed + initialConditions.vix * t + 0.5 * ax * Math.pow(t, 2) + dxConstantSpeed,
					initialConditions.diy + dyBeforeMaxSpeed + initialConditions.viy * t + 0.5 * ay * Math.pow(t, 2) + dyConstantSpeed));
		currentState.setVelocity(vx, vy);
	}
	
	private void updateRunningToStationary() {
		double vfx = 0;
		double vfy = 0;
		double dxBeforeMinSpeed = 0;
		double dyBeforeMinSpeed = 0;
		double dxConstantSpeed = 0;
		double dyConstantSpeed = 0;
		double tBeforeMinSpeed = 0;
		// currently running, use deceleration to stop
		double vi = initialConditions.getInitialVelocity();
		double vf = Math.sqrt(Math.pow(vfx, 2) + Math.pow(vfy, 2));
		double desiredTime = goalState.getDesiredTime() == -1 ? (2 * initialConditions.getTotalDistance()) / (vf + vi) : goalState.getDesiredTime();
		//calculate acceleration
		ax = (initialConditions.dx - initialConditions.vix * desiredTime) * 2.0 / (Math.pow(desiredTime, 2));
		ay = (initialConditions.dy - initialConditions.viy * desiredTime) * 2.0 / (Math.pow(desiredTime, 2));
		
		double a = Math.sqrt(Math.pow(ax, 2) + Math.pow(ay, 2));
		if (a > maxDeceleration) {
			// account for maximum acceleration
			ax = ax * maxDeceleration / a;
			ay = ay * maxDeceleration / a;
		}
		// find velocity
		double vx = initialConditions.vix + ax * t;
		double vy = initialConditions.viy + ay * t;
		double dxNoAdjustment = initialConditions.vix * t + 0.5 * ax * Math.pow(t, 2);
		double dyNoAdjustment = initialConditions.viy * t + 0.5 * ay * Math.pow(t, 2);
		if ((vx < 0 && initialConditions.vix > 0) || (vx > 0 && initialConditions.vix < 0)) {
			vx = 0;
			//vf^2 = vi^2 + 2ad
			dxBeforeMinSpeed = -Math.pow(initialConditions.vix, 2) / 2.0 / ax;
			ax = 0;
			dxNoAdjustment = 0;
		}
		if ((vy < 0 && initialConditions.viy > 0) || (vy > 0 && initialConditions.viy < 0)) {
			vy = 0;
			dyBeforeMinSpeed = -Math.pow(initialConditions.viy, 2) / 2.0 / ay;
			ay = 0;
			dyNoAdjustment = 0;
		}
		currentState.setPosition(new Position(initialConditions.dix + dxBeforeMinSpeed + dxNoAdjustment + dxConstantSpeed,
				initialConditions.diy + dyBeforeMinSpeed + dyNoAdjustment + dyConstantSpeed));
		currentState.setVelocity(vx, vy);
	}

}
