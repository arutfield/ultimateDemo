package mainwindow;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.Shape;
import java.awt.Toolkit;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.geom.Ellipse2D;
import java.util.HashMap;
import java.util.Map;

import javax.swing.ImageIcon;
import javax.swing.JPanel;
import javax.swing.Timer;

import model.Player;
import utilities.PlayerNameEnum;
import utilities.Position;

public class MainViewer extends JPanel implements ActionListener {

	private final int B_WIDTH = 500;
	private final int B_HEIGHT = 500;
	private final int INITIAL_X = -40;
	private final int INITIAL_Y = -40;
	private final int DELAY = 100;

	private Timer timer;
	private int x, y;
	private long previousTimeStamp = System.currentTimeMillis();
	private Map<PlayerNameEnum, Player> playerMap = new HashMap<>();
	
	public MainViewer() {

		initMainViewer();
	}

	private void initMainViewer() {

		setBackground(Color.WHITE);
		setPreferredSize(new Dimension(B_WIDTH, B_HEIGHT));

		x = INITIAL_X;
		y = INITIAL_Y;
		setUpPlayers();
		timer = new Timer(DELAY, this);
		timer.start();
	}

	private void setUpPlayers() {
		for (PlayerNameEnum playerName: PlayerNameEnum.values()) {
			Player player = new Player(new Position(1, 1), 7.0, 9.0, 20.0, 20.0);
			playerMap.put(playerName, player);
		}
	}

	@Override
	public void paintComponent(Graphics g) {
		super.paintComponent(g);
		drawPlayers(g);
		updateTimer();
	}

	private void updateTimer() {
		long currentTime = System.currentTimeMillis();
		//System.out.println(currentTime - previousTimeStamp);
		long timeDifference = currentTime - previousTimeStamp;
		int error = (int) (timeDifference - DELAY);
		int adjustedDelay = (int) (DELAY - 0.8*error);
		if (adjustedDelay < 0) {
			adjustedDelay = 1;
		}
		previousTimeStamp = System.currentTimeMillis();
		timer.setDelay(adjustedDelay);

	}

	private void drawPlayers(Graphics g) {

		Graphics2D ga = (Graphics2D) g;
		for (Player player : playerMap.values()) {
			player.update();
			Position position = player.getCurrentPosition();
			Shape circle = new Ellipse2D.Float((int) position.getX(), (int) position.getY(), 10.0f, 10.0f);
			ga.draw(circle);
			ga.setPaint(Color.green);
			ga.fill(circle);
		}

		Toolkit.getDefaultToolkit().sync();
	}

	@Override
	public void actionPerformed(ActionEvent e) {

		x += 1;
		y += 1;
		if (y > B_HEIGHT) {

			y = INITIAL_Y;
			x = INITIAL_X;
		}

		repaint();
	}

}
