package tests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
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
	double initialOffsetX = 3.2;
	double initialOffsetY = -2.5;
	double t = 0.1;
	
	private Player createPlayer() {
		double x = 0.0;
		double y = 0.0;
		Position initialPosition = new Position(x, y);
		return new Player(initialPosition, runningSpeed, maxSpeed, maxAcceleration, maxDeceleration);
	}
	
	private Player createPlayerOffset() {
		Position initialPosition = new Position(initialOffsetX, initialOffsetY);
		return new Player(initialPosition, runningSpeed, maxSpeed, maxAcceleration, maxDeceleration);
	}

	private void checkGoals(Player player, double goalX, double goalY, double velX, double velY) {
		System.out.println(goalX + "," + goalY + "," + velX + "," + velY);
		assertEquals(goalX, player.getCurrentPosition().getX(), 1e-5);
		assertEquals(goalY, player.getCurrentPosition().getY(), 1e-5);	
		assertEquals(velX, player.getVelocityX(), 1e-5);
		assertEquals(velY, player.getVelocityY(), 1e-5);
	}
	
	@Test
	public void testTest() {
		assertEquals(true, true);
	}

	@Test
	public void testInitialPosition() {
		Player player = createPlayer();
		checkGoals(player, 0.0, 0.0, 0.0, 0.0);
	}
	
	@Test
	public void testGoalPositionFirstUpdateInitialToRunningX() {
		Player player = createPlayer();
		player.setGoalState(new GoalState(new Position(5, 0), FinalSpeedEnum.Running));
		player.update();
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2
		checkGoals(player, 0.5*maxAcceleration*t*t, 0, maxAcceleration*t, 0.0);
	}
	
	@Test
	public void testGoalPositionFirstUpdateInitialToRunningY() {
		Player player = createPlayer();
		player.setGoalState(new GoalState(new Position(0, 5), FinalSpeedEnum.Running));
		player.update();
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2
		checkGoals(player, 0, 0.5*maxAcceleration*t*t, 0.0, maxAcceleration*t);
	}

	@Test
	public void testGoalPositionFirstUpdateInitialToRunningNegativeX() {
		Player player = createPlayer();
		player.setGoalState(new GoalState(new Position(-5, 0), FinalSpeedEnum.Running));
		player.update();
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2		
		checkGoals(player, -0.5*maxAcceleration*t*t, 0, -maxAcceleration*t, 0.0);
	}
	
	@Test
	public void testGoalPositionFirstUpdateInitialToRunningNegativeY() {
		Player player = createPlayer();
		player.setGoalState(new GoalState(new Position(0, -5), FinalSpeedEnum.Running));
		player.update();
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2		
		checkGoals(player, 0, -0.5*maxAcceleration*t*t, 0.0, -maxAcceleration*t);
	}

	
	@Test
	public void testGoalPositionFirstUpdateInitialToRunningXY() {
		Player player = createPlayer();
		Position goal = new Position(5, 5);
		double angle = Math.atan2(goal.getY(), goal.getX());
		player.setGoalState(new GoalState(goal, FinalSpeedEnum.Running));
		player.update();
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2
		
		checkGoals(player, 0.5*maxAcceleration*Math.cos(angle)*t*t,
				0.5*maxAcceleration*Math.sin(angle)*t*t, maxAcceleration*t*Math.cos(angle),
				maxAcceleration*t*Math.sin(angle));
	}

	@Test
	public void testGoalPositionFirstUpdateInitialToRunningNegativeXY() {
		Player player = createPlayer();
		Position goal = new Position(-5, -5);
		double angle = Math.atan2(goal.getY(), goal.getX());
		player.setGoalState(new GoalState(goal, FinalSpeedEnum.Running));
		player.update();
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2		
		checkGoals(player, 0.5*maxAcceleration*Math.cos(angle)*t*t,
				0.5*maxAcceleration*Math.sin(angle)*t*t, maxAcceleration*Math.cos(angle)*t,
				maxAcceleration*Math.sin(angle)*t);
	}
	
	@Test
	public void testGoalPositionFirstUpdateInitialToRunningXYDesiredTime() {
		Player player = createPlayer();
		Position goal = new Position(5, 5);
		double angle = Math.atan2(goal.getY(), goal.getX());
		player.setGoalState(new GoalState(goal, FinalSpeedEnum.Running, 5));
		player.update();
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2
		
		checkGoals(player, 0.5*0.4*t*t, 0.5*0.4*t*t, 0.4*t, 0.4*t);
	}
	
	@Test
	public void testGoalPositionFirstUpdateInitialToRunningXYMaxAcceleration() {
		Player player = createPlayer();
		Position goal = new Position(5e5, 5e5);
		double angle = Math.atan2(goal.getY(), goal.getX());
		player.setGoalState(new GoalState(goal, FinalSpeedEnum.Running, 1));
		player.update();
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2
		
		checkGoals(player, 0.5*maxAcceleration*Math.cos(angle)*t*t,
				0.5*maxAcceleration*Math.sin(angle)*t*t, maxAcceleration*Math.cos(angle)*t,
				maxAcceleration*Math.sin(angle)*t);
	}

	@Test
	public void testGoalPositionFirstUpdateInitialToRunningXYMaxAccelerationAllSteps() {
		Player player = createPlayer();
		Position goal = new Position(5e4, 5e4);
		double angle = Math.atan2(goal.getY(), goal.getX());
		player.setGoalState(new GoalState(goal, FinalSpeedEnum.Running, 1));
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2
		//v^2 = v0^2+2ax
		double distanceBeforeMaxSpeed = (Math.pow(maxSpeed, 2))/(2*maxAcceleration);
		double timeBeforeMaxSpeed = maxSpeed / maxAcceleration;
		for (double i=0.1; i < 1000; i = i+0.1) {
			player.update();
			
			double goalY = 0.5*maxAcceleration*Math.sin(angle)*i*i;
			double goalX = 0.5*maxAcceleration*Math.cos(angle)*i*i;
			double velX = maxAcceleration*Math.cos(angle)*i;
			double velY = maxAcceleration*Math.sin(angle)*i;
			if (Math.sqrt(Math.pow(goalX, 2) + Math.pow(goalY, 2)) > distanceBeforeMaxSpeed) {
				goalX = (distanceBeforeMaxSpeed + (i - timeBeforeMaxSpeed)*maxSpeed) * Math.cos(angle);
				goalY = (distanceBeforeMaxSpeed + (i - timeBeforeMaxSpeed)*maxSpeed) * Math.sin(angle);
				velX = maxSpeed*Math.cos(angle);
				velY = maxSpeed*Math.sin(angle);
			}
			checkGoals(player, goalX, goalY, velX, velY);
		}
	}
	
	@Test
	public void testGoalPositionFirstUpdateInitialToRunningXYMaxAccelerationAllStepsLongTriangle() {
		Player player = createPlayer();
		Position goal = new Position(5e4, 1e4);
		double angle = Math.atan2(goal.getY(), goal.getX());
		player.setGoalState(new GoalState(goal, FinalSpeedEnum.Running, 1));
		
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2
		//v^2 = v0^2+2ax
		double distanceBeforeMaxSpeed = (Math.pow(maxSpeed, 2))/(2*maxAcceleration);
		double timeBeforeMaxSpeed = maxSpeed / maxAcceleration;
		for (double i=0.1; i < 1000; i = i+0.1) {
			player.update();
			
			double goalX = 0.5*maxAcceleration*Math.cos(angle)*i*i;
			double goalY = 0.5*maxAcceleration*Math.sin(angle)*i*i;
			double velX = maxAcceleration*Math.cos(angle)*i;
			double velY = maxAcceleration*Math.sin(angle)*i;
			if (Math.sqrt(Math.pow(goalX, 2) + Math.pow(goalY, 2)) > distanceBeforeMaxSpeed) {
				goalX = (distanceBeforeMaxSpeed + (i - timeBeforeMaxSpeed)*maxSpeed) * Math.cos(angle);
				goalY = (distanceBeforeMaxSpeed + (i - timeBeforeMaxSpeed)*maxSpeed) * Math.sin(angle);
				velX = maxSpeed*Math.cos(angle);
				velY = maxSpeed*Math.sin(angle);
			}
			checkGoals(player, goalX, goalY, velX, velY);
		}
	}

	//player stationary to running with initial offset
	@Test
	public void testInitialPositionOffset() {
		Player player = createPlayerOffset();
		checkGoals(player, initialOffsetX, initialOffsetY, 0.0, 0.0);
	}
	
	@Test
	public void testGoalPositionFirstUpdateInitialToRunningXOffset() {
		Player player = createPlayerOffset();
		player.setGoalState(new GoalState(new Position(5, initialOffsetY), FinalSpeedEnum.Running));
		player.update();
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2		
		checkGoals(player, initialOffsetX + 0.5*maxAcceleration*t*t, initialOffsetY, maxAcceleration*t, 0.0);
	}
	
	@Test
	public void testGoalPositionFirstUpdateInitialToRunningYOffset() {
		Player player = createPlayerOffset();
		player.setGoalState(new GoalState(new Position(initialOffsetX, 5), FinalSpeedEnum.Running));
		player.update();
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2		
		checkGoals(player, initialOffsetX, initialOffsetY + 0.5*maxAcceleration*t*t, 0.0, maxAcceleration*0.1);
	}

	@Test
	public void testGoalPositionFirstUpdateInitialToRunningNegativeXOffset() {
		Player player = createPlayerOffset();
		player.setGoalState(new GoalState(new Position(-5, initialOffsetY), FinalSpeedEnum.Running));
		player.update();
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2		
		checkGoals(player, initialOffsetX-0.5*maxAcceleration*t*t, initialOffsetY, -maxAcceleration*t, 0.0);
	}
	
	@Test
	public void testGoalPositionFirstUpdateInitialToRunningNegativeYOffset() {
		Player player = createPlayerOffset();
		player.setGoalState(new GoalState(new Position(initialOffsetX, -5), FinalSpeedEnum.Running));
		player.update();
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2		
		checkGoals(player, initialOffsetX, initialOffsetY-0.5*maxAcceleration*t*t, 0.0, -maxAcceleration*t);
	}

	
	@Test
	public void testGoalPositionFirstUpdateInitialToRunningXYOffset() {
		Player player = createPlayerOffset();
		player.setGoalState(new GoalState(new Position(5, 5), FinalSpeedEnum.Running));
		double angle = Math.atan((5-initialOffsetY)/(5 - initialOffsetX));
		player.update();
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2		
		checkGoals(player, initialOffsetX + 0.5*maxAcceleration*Math.cos(angle)*t*t,
				initialOffsetY + 0.5*maxAcceleration*Math.sin(angle)*t*t,
				maxAcceleration*Math.cos(angle)*t, maxAcceleration*Math.sin(angle)*t);
	}

	@Test
	public void testGoalPositionFirstUpdateInitialToRunningNegativeXYOffset() {
		Player player = createPlayerOffset();
		player.setGoalState(new GoalState(new Position(-5, -5), FinalSpeedEnum.Running));
		double angle = Math.atan((-5 - initialOffsetY)/(-5 - initialOffsetX));
		player.update();
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2
		checkGoals(player, initialOffsetX-0.5*maxAcceleration*Math.cos(angle)*t*t,
				initialOffsetY-0.5*maxAcceleration*Math.sin(angle)*t*t,
				-maxAcceleration*Math.cos(angle)*t, -maxAcceleration*Math.sin(angle)*t);
	}
	
	@Test
	public void testGoalPositionFirstUpdateInitialToRunningXYDesiredTimeOffset() {
		Player player = createPlayerOffset();
		player.setGoalState(new GoalState(new Position(5, 5), FinalSpeedEnum.Running, 5));
		double angle = Math.atan((5 - initialOffsetY)/(5 - initialOffsetX));
		double distance = Math.sqrt(Math.pow(5 - initialOffsetX, 2) + Math.pow(5 - initialOffsetY, 2));
		player.update();
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2
		double acceleration = distance/0.5/5/5;
		
		checkGoals(player, initialOffsetX + 0.5*acceleration*Math.cos(angle)*t*t,
				initialOffsetY + 0.5*acceleration*Math.sin(angle)*t*t,
				acceleration*Math.cos(angle)*t, acceleration*Math.sin(angle)*t);
	}
	
	@Test
	public void testGoalPositionFirstUpdateInitialToRunningXYMaxAccelerationOffset() {
		Player player = createPlayerOffset();
		player.setGoalState(new GoalState(new Position(5e5, 5e5), FinalSpeedEnum.Running, 1));
		double angle = Math.atan((5e5 - initialOffsetY)/(5e5 - initialOffsetX));
		player.update();
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2
		checkGoals(player, initialOffsetX + 0.5*maxAcceleration*Math.cos(angle)*t*t,
				initialOffsetY + 0.5*maxAcceleration*Math.sin(angle)*t*t,
				maxAcceleration*Math.cos(angle)*t, maxAcceleration*Math.sin(angle)*t);
	}

	@Test
	public void testGoalPositionFirstUpdateInitialToRunningXYMaxAccelerationAllStepsOffset() {
		Player player = createPlayerOffset();
		player.setGoalState(new GoalState(new Position(5e4, 5e4), FinalSpeedEnum.Running, 1));
		double angle = Math.atan((5e4 - initialOffsetY)/(5e4 - initialOffsetX));
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2
		//v^2 = v0^2+2ax
		double distanceBeforeMaxSpeed = (Math.pow(maxSpeed, 2))/(2*maxAcceleration);
		double timeBeforeMaxSpeed = maxSpeed / maxAcceleration;
		for (double i=0.1; i < 1000; i = i+0.1) {
			player.update();
			
			double goalY = initialOffsetY + 0.5*maxAcceleration*Math.sin(angle)*i*i;
			double goalX = initialOffsetX + 0.5*maxAcceleration*Math.cos(angle)*i*i;
			double velY = maxAcceleration*Math.sin(angle)*i;
			double velX = maxAcceleration*Math.cos(angle)*i;
			if (Math.sqrt(Math.pow(goalX-initialOffsetX, 2) + Math.pow(goalY-initialOffsetY, 2)) > distanceBeforeMaxSpeed) {
				goalX = initialOffsetX + (distanceBeforeMaxSpeed + (i - timeBeforeMaxSpeed)*maxSpeed) * Math.cos(angle);
				goalY = initialOffsetY + (distanceBeforeMaxSpeed + (i - timeBeforeMaxSpeed)*maxSpeed) * Math.sin(angle);
				velX = maxSpeed*Math.cos(angle);
				velY = maxSpeed*Math.sin(angle);
			}
			checkGoals(player, goalX, goalY, velX, velY);
		}
	}
	
	@Test
	public void testGoalPositionFirstUpdateInitialToRunningXYMaxAccelerationAllStepsLongTriangleOffset() {
		Player player = createPlayerOffset();
		player.setGoalState(new GoalState(new Position(1e4, 5e4), FinalSpeedEnum.Running, 1));
		double angle=Math.atan((5e4-initialOffsetY)/(1e4-initialOffsetX));
		
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2
		//v^2 = v0^2+2ax
		double distanceBeforeMaxSpeed = (Math.pow(maxSpeed, 2))/(2*maxAcceleration);
		double timeBeforeMaxSpeed = maxSpeed / maxAcceleration;
		for (double i=0.1; i < 1000; i = i+0.1) {
			player.update();
			
			double goalX = initialOffsetX + 0.5*maxAcceleration*Math.cos(angle)*i*i;
			double goalY = initialOffsetY + 0.5*maxAcceleration*Math.sin(angle)*i*i;
			double velY = maxAcceleration*Math.sin(angle)*i;
			double velX = maxAcceleration*Math.cos(angle)*i;
			if (Math.sqrt(Math.pow(goalX-initialOffsetX, 2) + Math.pow(goalY-initialOffsetY, 2)) > distanceBeforeMaxSpeed) {
				goalX = initialOffsetX + (distanceBeforeMaxSpeed + (i - timeBeforeMaxSpeed)*maxSpeed) * Math.cos(angle);
				goalY = initialOffsetY + (distanceBeforeMaxSpeed + (i - timeBeforeMaxSpeed)*maxSpeed) * Math.sin(angle);
				velX = maxSpeed*Math.cos(angle);
				velY = maxSpeed*Math.sin(angle);
			}
			checkGoals(player, goalX, goalY, velX, velY);
		}
	}

	//test code for going from running to stop
	@Test
	public void testGoalPositionFirstUpdateRunningToStopX() {
		Player player = createPlayer();
		double desX = 5;
		double desY = 0;
		double angle = Math.atan(desY / desX);
		player.setGoalState(new GoalState(new Position(desX, desY), FinalSpeedEnum.Running));
		double distanceBeforeMaxSpeed = (Math.pow(maxSpeed, 2)) / (2 * maxAcceleration);
		double timeBeforeMaxSpeed = maxSpeed / maxAcceleration;
		double goalX = 0;
		double goalY = 0;
		for (double i=0.1; i<5; i=i+0.1) {
			player.update();
			goalX = 0.5*maxAcceleration*Math.cos(angle)*i*i;
			goalY = 0.5*maxAcceleration*Math.sin(angle)*i*i;
			double velY = maxAcceleration*Math.sin(angle)*i;
			double velX = maxAcceleration*Math.cos(angle)*i;
			if (Math.sqrt(Math.pow(goalX, 2) + Math.pow(goalY, 2)) > distanceBeforeMaxSpeed) {
				goalX = (distanceBeforeMaxSpeed + (i - timeBeforeMaxSpeed)*maxSpeed) * Math.cos(angle);
				goalY = (distanceBeforeMaxSpeed + (i - timeBeforeMaxSpeed)*maxSpeed) * Math.sin(angle);
				velX = maxSpeed*Math.cos(angle);
				velY = maxSpeed*Math.sin(angle);

			}
			checkGoals(player, goalX, goalY, velX, velY);
		}
		
		Position startPosition = new Position(goalX, goalY);
		double initialVelocityX = player.getVelocityX();
		double initialVelocityY = player.getVelocityY();
		//start decelerating to stop
		double des2X = goalX + initialVelocityX*0.1;
		double des2Y = goalY + initialVelocityY*0.1;
		double angle2 = Math.atan(des2Y / des2X);
		Position goal = new Position(des2X, des2Y);
		player.setGoalState(new GoalState(goal, FinalSpeedEnum.Stationary));
		double distanceBeforeStopX = startPosition.getX() + (Math.pow(initialVelocityX, 2)) / (2 * maxDeceleration * Math.cos(angle));
		double distanceBeforeStopY = startPosition.getY() + (Math.pow(initialVelocityY, 2)) / (2 * maxDeceleration * Math.sin(angle));
		for (double i=0.1; i<10; i=i+0.1) {
			player.update();
			if (initialVelocityX*i > 0.5*maxDeceleration*Math.cos(angle2)*i*i)
				goalX = startPosition.getX() + initialVelocityX*i - 0.5*maxDeceleration*Math.cos(angle2)*i*i; 
				
			if (initialVelocityY*i > 0.5*maxDeceleration*Math.sin(angle2)*i*i)
				goalY = startPosition.getY() + initialVelocityY*i - 0.5*maxDeceleration*Math.sin(angle2)*i*i;
			double velX = initialVelocityX - maxDeceleration*Math.cos(angle2)*i;
			if (initialVelocityX*velX < 0) {
				velX = 0;
				goalX = distanceBeforeStopX;
			}
			double velY = initialVelocityY - maxDeceleration*Math.sin(angle2)*i;
			if (initialVelocityY*velY < 0) {
				velY = 0;
				goalY = distanceBeforeStopY;
			}
			checkGoals(player, goalX, goalY, velX, velY);
		}
		
		assertEquals(0.0, player.getVelocityX(), 1e-5);
		assertEquals(0.0, player.getVelocityY(), 1e-5);
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2
	}

	@Test
	public void testGoalPositionFirstUpdateRunningToStopY() {
		Player player = createPlayer();
		double desX = 0;
		double desY = 5;
		double angle = Math.atan(desY / desX);
		player.setGoalState(new GoalState(new Position(desX, desY), FinalSpeedEnum.Running));
		double distanceBeforeMaxSpeed = (Math.pow(maxSpeed, 2)) / (2 * maxAcceleration);
		double timeBeforeMaxSpeed = maxSpeed / maxAcceleration;
		double goalX = 0;
		double goalY = 0;
		for (double i=0.1; i<5; i=i+0.1) {
			player.update();
			goalX = 0.5*maxAcceleration*Math.cos(angle)*i*i;
			goalY = 0.5*maxAcceleration*Math.sin(angle)*i*i;
			double velY = maxAcceleration*Math.sin(angle)*i;
			double velX = maxAcceleration*Math.cos(angle)*i;
			if (Math.sqrt(Math.pow(goalX, 2) + Math.pow(goalY, 2)) > distanceBeforeMaxSpeed) {
				goalX = (distanceBeforeMaxSpeed + (i - timeBeforeMaxSpeed)*maxSpeed) * Math.cos(angle);
				goalY = (distanceBeforeMaxSpeed + (i - timeBeforeMaxSpeed)*maxSpeed) * Math.sin(angle);
				velX = maxSpeed*Math.cos(angle);
				velY = maxSpeed*Math.sin(angle);

			}
			checkGoals(player, goalX, goalY, velX, velY);
		}
		
		Position startPosition = new Position(goalX, goalY);
		double initialVelocityX = player.getVelocityX();
		double initialVelocityY = player.getVelocityY();
		//start decelerating to stop
		double des2X = goalX + initialVelocityX*0.1;
		double des2Y = goalY + initialVelocityY*0.1;
		double angle2 = Math.atan(des2Y / des2X);
		Position goal = new Position(des2X, des2Y);
		player.setGoalState(new GoalState(goal, FinalSpeedEnum.Stationary));
		double distanceBeforeStopX = startPosition.getX() + (Math.pow(initialVelocityX, 2)) / (2 * maxDeceleration * Math.cos(angle));
		double distanceBeforeStopY = startPosition.getY() + (Math.pow(initialVelocityY, 2)) / (2 * maxDeceleration * Math.sin(angle));
		for (double i=0.1; i<10; i=i+0.1) {
			player.update();
			if (initialVelocityX*i > 0.5*maxDeceleration*Math.cos(angle2)*i*i)
				goalX = startPosition.getX() + initialVelocityX*i - 0.5*maxDeceleration*Math.cos(angle2)*i*i; 
				
			if (initialVelocityY*i > 0.5*maxDeceleration*Math.sin(angle2)*i*i)
				goalY = startPosition.getY() + initialVelocityY*i - 0.5*maxDeceleration*Math.sin(angle2)*i*i;
			double velX = initialVelocityX - maxDeceleration*Math.cos(angle2)*i;
			if (initialVelocityX*velX < 0) {
				velX = 0;
				goalX = distanceBeforeStopX;
			}
			double velY = initialVelocityY - maxDeceleration*Math.sin(angle2)*i;
			if (initialVelocityY*velY < 0) {
				velY = 0;
				goalY = distanceBeforeStopY;
			}
			checkGoals(player, goalX, goalY, velX, velY);
		}
		
		assertEquals(0.0, player.getVelocityX(), 1e-5);
		assertEquals(0.0, player.getVelocityY(), 1e-5);
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2
	}

	@Test
	public void testGoalPositionFirstUpdateRunningToStopXY() {
		Player player = createPlayer();
		double desX = 8;
		double desY = 5;
		double angle = Math.atan(desY / desX);
		player.setGoalState(new GoalState(new Position(desX, desY), FinalSpeedEnum.Running));
		double distanceBeforeMaxSpeed = (Math.pow(maxSpeed, 2)) / (2 * maxAcceleration);
		double timeBeforeMaxSpeed = maxSpeed / maxAcceleration;
		double goalX = 0;
		double goalY = 0;
		for (double i=0.1; i<5; i=i+0.1) {
			player.update();
			goalX = 0.5*maxAcceleration*Math.cos(angle)*i*i;
			goalY = 0.5*maxAcceleration*Math.sin(angle)*i*i;
			double velY = maxAcceleration*Math.sin(angle)*i;
			double velX = maxAcceleration*Math.cos(angle)*i;
			if (Math.sqrt(Math.pow(goalX, 2) + Math.pow(goalY, 2)) > distanceBeforeMaxSpeed) {
				goalX = (distanceBeforeMaxSpeed + (i - timeBeforeMaxSpeed)*maxSpeed) * Math.cos(angle);
				goalY = (distanceBeforeMaxSpeed + (i - timeBeforeMaxSpeed)*maxSpeed) * Math.sin(angle);
				velX = maxSpeed*Math.cos(angle);
				velY = maxSpeed*Math.sin(angle);

			}
			checkGoals(player, goalX, goalY, velX, velY);
		}
		
		Position startPosition = new Position(goalX, goalY);
		double initialVelocityX = player.getVelocityX();
		double initialVelocityY = player.getVelocityY();
		//start decelerating to stop
		double des2X = goalX + initialVelocityX*0.1;
		double des2Y = goalY + initialVelocityY*0.1;
		double angle2 = Math.atan(des2Y / des2X);
		Position goal = new Position(des2X, des2Y);
		player.setGoalState(new GoalState(goal, FinalSpeedEnum.Stationary));
		double distanceBeforeStopX = startPosition.getX() + (Math.pow(initialVelocityX, 2)) / (2 * maxDeceleration * Math.cos(angle));
		double distanceBeforeStopY = startPosition.getY() + (Math.pow(initialVelocityY, 2)) / (2 * maxDeceleration * Math.sin(angle));
		for (double i=0.1; i<10; i=i+0.1) {
			player.update();
			if (initialVelocityX*i > 0.5*maxDeceleration*Math.cos(angle2)*i*i)
				goalX = startPosition.getX() + initialVelocityX*i - 0.5*maxDeceleration*Math.cos(angle2)*i*i; 
				
			if (initialVelocityY*i > 0.5*maxDeceleration*Math.sin(angle2)*i*i)
				goalY = startPosition.getY() + initialVelocityY*i - 0.5*maxDeceleration*Math.sin(angle2)*i*i;
			double velX = initialVelocityX - maxDeceleration*Math.cos(angle2)*i;
			if (initialVelocityX*velX < 0) {
				velX = 0;
				goalX = distanceBeforeStopX;
			}
			double velY = initialVelocityY - maxDeceleration*Math.sin(angle2)*i;
			if (initialVelocityY*velY < 0) {
				velY = 0;
				goalY = distanceBeforeStopY;
			}
			checkGoals(player, goalX, goalY, velX, velY);
		}
		
		assertEquals(0.0, player.getVelocityX(), 1e-5);
		assertEquals(0.0, player.getVelocityY(), 1e-5);
		//kinematic equation: x=v_0x*t+0.5*a_x*t^2
	}
}
