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
	public void setVelocityX(double velocityX) {
		this.velocityX = velocityX;
	}
	public double getVelocityY() {
		return velocityY;
	}
	public void setVelocityY(double velocityY) {
		this.velocityY = velocityY;
	}
	
	public double getVelocity() {
		return Math.sqrt(Math.pow(velocityX, 2) + Math.pow(velocityY, 2));
	}
	

}
