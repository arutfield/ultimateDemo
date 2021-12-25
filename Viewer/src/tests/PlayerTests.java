package tests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.jupiter.api.Test;

import model.Player;
import utilities.FinalSpeedEnum;
import utilities.GoalState;
import utilities.Position;

public class PlayerTests {
	double maxSpeed = 10;
	double runningSpeed = 8;
	double maxAcceleration = 10;
	double maxDeceleration = 15;

	private Player createPlayer() {
		double x = 0.0;
		double y = 0.0;
		Position initialPosition = new Position(x, y);
		return new Player(initialPosition, runningSpeed, maxSpeed, maxAcceleration, maxDeceleration);
	}
	
	@Test
	public void testTest() {
		assertEquals(true, true);
	}

	@Test
	public void testInitialPosition() {
		Player player = createPlayer();
		assertEquals(player.getCurrentPosition().getX(), 0.0, 1e-5);
		assertEquals(player.getCurrentPosition().getY(), 0.0, 1e-5);
	}
	
	@Test
	public void testGoalPositionFirstUpdateInitialToRunningX() {
		Player player = createPlayer();
		player.setGoalState(new GoalState(new Position(5, 0), FinalSpeedEnum.Running));
		player.update();
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2
		
		double goalX = 0.5*maxAcceleration*0.1*0.1;
		double goalY = 0;
		assertEquals(goalX, player.getCurrentPosition().getX(), 1e-5);
		assertEquals(goalY, player.getCurrentPosition().getY(), 1e-5);
	}
	
	@Test
	public void testGoalPositionFirstUpdateInitialToRunningY() {
		Player player = createPlayer();
		player.setGoalState(new GoalState(new Position(0, 5), FinalSpeedEnum.Running));
		player.update();
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2
		
		double goalY = 0.5*maxAcceleration*0.1*0.1;
		double goalX = 0;
		assertEquals(goalX, player.getCurrentPosition().getX(), 1e-5);
		assertEquals(goalY, player.getCurrentPosition().getY(), 1e-5);
	}

	@Test
	public void testGoalPositionFirstUpdateInitialToRunningNegativeX() {
		Player player = createPlayer();
		player.setGoalState(new GoalState(new Position(-5, 0), FinalSpeedEnum.Running));
		player.update();
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2
		
		double goalX = -0.5*maxAcceleration*0.1*0.1;
		double goalY = 0;
		assertEquals(goalX, player.getCurrentPosition().getX(), 1e-5);
		assertEquals(goalY, player.getCurrentPosition().getY(), 1e-5);
	}
	
	@Test
	public void testGoalPositionFirstUpdateInitialToRunningNegativeY() {
		Player player = createPlayer();
		player.setGoalState(new GoalState(new Position(0, -5), FinalSpeedEnum.Running));
		player.update();
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2
		
		double goalY = -0.5*maxAcceleration*0.1*0.1;
		double goalX = 0;
		assertEquals(goalX, player.getCurrentPosition().getX(), 1e-5);
		assertEquals(goalY, player.getCurrentPosition().getY(), 1e-5);
	}

	
	@Test
	public void testGoalPositionFirstUpdateInitialToRunningXY() {
		Player player = createPlayer();
		player.setGoalState(new GoalState(new Position(5, 5), FinalSpeedEnum.Running));
		player.update();
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2
		
		double goalY = 0.5*maxAcceleration*Math.sin(45.0 * Math.PI / 180.0)*0.1*0.1;
		double goalX = 0.5*maxAcceleration*Math.cos(45.0 * Math.PI / 180.0)*0.1*0.1;
		assertEquals(goalX, player.getCurrentPosition().getX(), 1e-5);
		assertEquals(goalY, player.getCurrentPosition().getY(), 1e-5);
	}

	@Test
	public void testGoalPositionFirstUpdateInitialToRunningNegativeXY() {
		Player player = createPlayer();
		player.setGoalState(new GoalState(new Position(-5, -5), FinalSpeedEnum.Running));
		player.update();
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2
		
		double goalY = -0.5*maxAcceleration*Math.sin(45.0 * Math.PI / 180.0)*0.1*0.1;
		double goalX = -0.5*maxAcceleration*Math.cos(45.0 * Math.PI / 180.0)*0.1*0.1;
		assertEquals(goalX, player.getCurrentPosition().getX(), 1e-5);
		assertEquals(goalY, player.getCurrentPosition().getY(), 1e-5);
	}
	
	@Test
	public void testGoalPositionFirstUpdateInitialToRunningXYDesiredTime() {
		Player player = createPlayer();
		player.setGoalState(new GoalState(new Position(5, 5), FinalSpeedEnum.Running, 5));
		player.update();
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2
		
		double goalY = 0.5*0.4*0.1*0.1;
		double goalX = 0.5*0.4*0.1*0.1;
		assertEquals(goalX, player.getCurrentPosition().getX(), 1e-5);
		assertEquals(goalY, player.getCurrentPosition().getY(), 1e-5);
	}
	
	@Test
	public void testGoalPositionFirstUpdateInitialToRunningXYMaxAcceleration() {
		Player player = createPlayer();
		player.setGoalState(new GoalState(new Position(5e5, 5e5), FinalSpeedEnum.Running, 1));
		player.update();
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2
		
		double goalY = 0.5*maxAcceleration*Math.sin(45.0 * Math.PI / 180.0)*0.1*0.1;
		double goalX = 0.5*maxAcceleration*Math.cos(45.0 * Math.PI / 180.0)*0.1*0.1;
		assertEquals(goalX, player.getCurrentPosition().getX(), 1e-5);
		assertEquals(goalY, player.getCurrentPosition().getY(), 1e-5);
	}

	@Test
	public void testGoalPositionFirstUpdateInitialToRunningXYMaxAccelerationAllSteps() {
		Player player = createPlayer();
		player.setGoalState(new GoalState(new Position(5e4, 5e4), FinalSpeedEnum.Running, 1));
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2
		//v^2 = v0^2+2ax
		double distanceBeforeMaxSpeed = (Math.pow(maxSpeed, 2))/(2*maxAcceleration);
		double timeBeforeMaxSpeed = maxSpeed / maxAcceleration;
		for (double i=0.1; i < 1000; i = i+0.1) {
			player.update();
			
			double goalY = 0.5*maxAcceleration*Math.sin(45.0 * Math.PI / 180.0)*i*i;
			double goalX = 0.5*maxAcceleration*Math.cos(45.0 * Math.PI / 180.0)*i*i;
			if (Math.sqrt(Math.pow(goalX, 2) + Math.pow(goalY, 2)) > distanceBeforeMaxSpeed) {
				goalX = (distanceBeforeMaxSpeed + (i - timeBeforeMaxSpeed)*maxSpeed) * Math.sin(45.0 * Math.PI / 180.0);
				goalY = (distanceBeforeMaxSpeed + (i - timeBeforeMaxSpeed)*maxSpeed) * Math.cos(45.0 * Math.PI / 180.0);
			}
			//System.out.println("goalX: " + goalX);
			assertEquals(goalX, player.getCurrentPosition().getX(), 1e-5);
			assertEquals(goalY, player.getCurrentPosition().getY(), 1e-5);
		}
	}
	
	@Test
	public void testGoalPositionFirstUpdateInitialToRunningXYMaxAccelerationAllStepsLongTriangle() {
		Player player = createPlayer();
		player.setGoalState(new GoalState(new Position(1e4, 5e4), FinalSpeedEnum.Running, 1));
		double angle=Math.atan(5e4/1e4);
		
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2
		//v^2 = v0^2+2ax
		double distanceBeforeMaxSpeed = (Math.pow(maxSpeed, 2))/(2*maxAcceleration);
		double timeBeforeMaxSpeed = maxSpeed / maxAcceleration;
		for (double i=0.1; i < 1000; i = i+0.1) {
			player.update();
			
			double goalX = 0.5*maxAcceleration*Math.cos(angle)*i*i;
			double goalY = 0.5*maxAcceleration*Math.sin(angle)*i*i;
			if (Math.sqrt(Math.pow(goalX, 2) + Math.pow(goalY, 2)) > distanceBeforeMaxSpeed) {
				goalX = (distanceBeforeMaxSpeed + (i - timeBeforeMaxSpeed)*maxSpeed) * Math.cos(angle);
				goalY = (distanceBeforeMaxSpeed + (i - timeBeforeMaxSpeed)*maxSpeed) * Math.sin(angle);
			}
			System.out.println("goalX: " + goalX);
			assertEquals(goalX, player.getCurrentPosition().getX(), 1e-5);
			assertEquals(goalY, player.getCurrentPosition().getY(), 1e-5);
		}
	}

}
