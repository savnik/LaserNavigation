/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package marg.gui.plugin.smr;

import java.awt.BorderLayout;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.RenderingHints;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;
import marg.images.ResUtils;

/**
 *
 * @author Daniel
 */
public class RobotWheel extends JPanel {

    private static final double WHEEL_SCALE = 0.5;
    private Image wheelImg;
    private double angle = 0;

    public RobotWheel() {
        super();
        this.setOpaque(false);
        wheelImg = ResUtils.getBufferedImage("SMRWheel.gif");
    }

    public void moveWheel(double degrees) {
        double newAngle = angle + degrees;
        if (newAngle > 360) {
            angle = newAngle % 360;
        } else if (newAngle < 0) {
            angle = 360 + (newAngle % 360);
        } else {
            angle = newAngle;
        }
        //System.out.println("Angle is "+ angle);
        this.repaint();
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        int centerX = this.getWidth() / 2;
        int centerY = this.getHeight() / 2;
        int x = centerX - (wheelImg.getWidth(null) / 4);
        int y = centerY - (wheelImg.getHeight(null) / 4);

        int width = (int) (wheelImg.getWidth(null) * WHEEL_SCALE);
        int height = (int) (wheelImg.getHeight(null) * WHEEL_SCALE);

        int wheelX = x + (wheelImg.getWidth(null) / 4);
        int wheelY = y + (wheelImg.getHeight(null) / 4);
        //System.out.println("Wheel angle is " + angle);

        //g2.drawString("X", wheelX, wheelY); //Draws X on center
        g2.rotate(angle * Math.PI / 180.0, wheelX, wheelY);
        g2.drawImage(wheelImg, x, y, width, height, null);
        //g2.drawImage(wheelImg, null, null);
    }

    public static void main(String[] args) {
        JFrame f = new JFrame();
        f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        final RobotWheel robWheel = new RobotWheel();
        f.setLayout(new BorderLayout());
        f.add(robWheel, BorderLayout.CENTER);
        f.setSize(400, 400);
        f.setLocation(200, 200);
        f.setVisible(true);
        JButton test = new JButton("Test Forward");
        test.setSize(100, 20);
        f.add(test, BorderLayout.SOUTH);
        test.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent e) {
                robWheel.moveWheel(-10);
                robWheel.repaint();
            }
        });
    }
}
