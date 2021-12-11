package com.zetcode;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Image;
import java.awt.Toolkit;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import javax.swing.ImageIcon;
import javax.swing.JPanel;
import javax.swing.Timer;
import java.awt.*;
import java.awt.event.*;
import java.awt.geom.*;

public class Board extends JPanel
        implements ActionListener {

    private final int B_WIDTH = 500;
    private final int B_HEIGHT = 500;
    private final int INITIAL_X = -40;
    private final int INITIAL_Y = -40;
    private final int DELAY = 100;

    private Image star;
    private Timer timer;
    private int x, y;
    private long[] lastTime = new long[10];
    private long previousTimeStamp = System.currentTimeMillis();
    private int lastTimeCounter = 0;
    public Board() {

        initBoard();
    }

    private void loadImage() {
    	System.out.println(System.getProperty("user.dir"));
        ImageIcon ii = new ImageIcon("src/resources/star.png");
        star = ii.getImage();
     }

    private void initBoard() {

        setBackground(Color.WHITE);
        setPreferredSize(new Dimension(B_WIDTH, B_HEIGHT));

        loadImage();
        
        x = INITIAL_X;
        y = INITIAL_Y;
        
        timer = new Timer(DELAY, this);
        timer.start();
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        drawStar(g);
        long currentTime = System.currentTimeMillis();
        System.out.println(currentTime - previousTimeStamp);
        lastTime[lastTimeCounter] = currentTime - previousTimeStamp;
        previousTimeStamp = System.currentTimeMillis();
        lastTimeCounter++;
        if (lastTimeCounter == 10) {
        	lastTimeCounter = 0;
        }
        double sum = 0;
        for (long period : lastTime) {
        	sum += (period-DELAY);
        }
        int averagePeriodError = (int) (sum/10.0);
        
        timer.setDelay(DELAY-averagePeriodError);
    }

    private void drawStar(Graphics g) {

        g.drawImage(star, x, y, this);
        Graphics2D ga = (Graphics2D)g;
        Shape circle = new Ellipse2D.Float(x, y, 100.0f, 100.0f);

        ga.draw(circle);
        ga.setPaint(Color.green);
        ga.fill(circle);
        ga.setPaint(Color.green);
        ga.draw(circle);
        Toolkit.getDefaultToolkit().sync();
    }

    @Override
    public void actionPerformed(ActionEvent e) {

        x += 1;
        y += 1;
        System.out.println("x: " + x + ", y: " + y);
        if (y > B_HEIGHT) {

            y = INITIAL_Y;
            x = INITIAL_X;
        }

        repaint();
    }
}