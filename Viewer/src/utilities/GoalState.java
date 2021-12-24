package utilities;

public class GoalState {
	
	private final Position position;
	private final FinalSpeedEnum finalSpeedEnum;
	private final double desiredTime;

	
	public GoalState(Position position, FinalSpeedEnum finalSpeedEnum, double desiredTime) {
		//TODO: error handling for invalid desired time
		this.position = position;
		this.finalSpeedEnum = finalSpeedEnum;
		this.desiredTime = desiredTime;
		
	}


	public GoalState(Position position, FinalSpeedEnum finalSpeedEnum) {
		this.position = position;
		this.finalSpeedEnum = finalSpeedEnum;
		this.desiredTime = -1;
	}


	public Position getPosition() {
		return position;
	}


	public FinalSpeedEnum getFinalSpeedEnum() {
		return finalSpeedEnum;
	}


	public double getDesiredTime() {
		return desiredTime;
	}
}
