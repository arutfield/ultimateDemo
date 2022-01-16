package utilities;

public class CurrentState {

	private Position position;
	private double velocityX; // m/s
	private double velocityY; // m/s
	
	public Position getPosition() {
		return position;
	}
	public void setPosition(Position position) {
		this.position = position;
	}
	public double getVelocityX() {
		return velocityX;
	}
	public double getVelocityY() {
		return velocityY;
	}
	
	public double getVelocity() {
		return Math.sqrt(Math.pow(velocityX, 2) + Math.pow(velocityY, 2));
	}
	public void setVelocity(double vx, double vy) {
		velocityX = vx;
		velocityY = vy;
	}
	

}
