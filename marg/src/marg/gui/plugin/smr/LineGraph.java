/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package marg.gui.plugin.smr;

import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Rectangle;
import java.awt.RenderingHints;
import java.awt.font.FontRenderContext;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;

/**
 *
 * @author Daniel
 */
public class LineGraph extends JPanel {

    private int[] data;
    private int maxValue = 10;
    private static final int PAD = 10;
    private static final int LABEL_BAR_HEIGHT = 15;
    private static final int SPACE_BETWEEN_BARS = 5;
    private static final int AXIS_WIDTH = 2;
    private Font font = new Font("Tahoma", Font.BOLD, 12);

    public LineGraph() {
        super();
        this.setBackground(new Color(233, 233, 237));
    }

    public LineGraph(int[] data) {
        super();
        this.setBackground(new Color(233, 233, 237));
        this.setData(data); //figures out maxValue itself
    }

    public LineGraph(int[] data, int maxValue) {
        super();
        this.setBackground(new Color(233, 233, 237));
        this.setData(data, maxValue);
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        paintBarGraph(g);
    }

    public void setData(int[] data, int maxValue) {
        this.data = data;
        this.maxValue = maxValue;
        //System.out.printf("data = %s%n", java.util.Arrays.toString(data));
        redrawLineGraph();
    }

    public void setMaxValue(int maxValue) {
        this.maxValue = maxValue;
    }

    public void setData(int[] data) {
        int highValue = 0;
        for (int i : data) {
            if (i > highValue) {
                highValue = i;
            }
        }
        this.data = data;
        this.maxValue = highValue + (int) (highValue * 0.2); //20% relative data higher
        //System.out.printf("data = %s%n", java.util.Arrays.toString(data));
        redrawLineGraph();
    }

    protected void paintBarGraph(Graphics g) {
        super.paintComponent(g);

        Graphics2D g2 = (Graphics2D) g;
        //Text Anti-aliasing: Disabled it for now as it didn't seem needed.
        //g2.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);

        int w = getWidth();
        int h = getHeight();
        g2.setPaint(new Color(50, 100, 133));
        // Draw ordinate.
        g2.fill(new Rectangle(PAD - AXIS_WIDTH, PAD, AXIS_WIDTH, h + 2 - (PAD * 2)- LABEL_BAR_HEIGHT)); //+2 so that it crosses the abcissa
        // Draw abcissa.
        g2.fill(new Rectangle(PAD, h - PAD - LABEL_BAR_HEIGHT, w - PAD * 2, AXIS_WIDTH));
        if (data != null) {
            double xInc = (double) (w - 2 * PAD) / data.length;
            double scale = (double) (h - 2 * PAD) / maxValue;
            // Draw data.
            double x = PAD;
            double y = h - PAD;
            int barWidth = (int) xInc - SPACE_BETWEEN_BARS;
            for (int i = 0; i < data.length; i++) {
                    x = PAD + i * xInc;
                    y = h - PAD - LABEL_BAR_HEIGHT - scale * data[i];
                    g2.setPaint(new Color(180, 180, 233));
                    double barHeightDouble = scale * data[i];
                    int barHeight = (int) Math.ceil(barHeightDouble);
                    g2.fill(new Rectangle((int) x + SPACE_BETWEEN_BARS, (int) y, barWidth, barHeight));

                    g2.setPaint(Color.BLACK);
                    g2.setFont(font);
                    FontRenderContext frc = g2.getFontRenderContext();
                    float barLabelWidth = (float) font.getStringBounds(String.valueOf(data[i]), frc).getWidth();
                    float labelWidth = (float) font.getStringBounds(String.valueOf(i), frc).getWidth();
                    //Draw Number
                    g2.drawString(String.valueOf(data[i]), (int) x + SPACE_BETWEEN_BARS + (barWidth / 2) - (barLabelWidth / 2), (int) y - 2);
                    //Draw Label
                    g2.drawString(String.valueOf(i+1), (int) x + SPACE_BETWEEN_BARS + (barWidth / 2) - (labelWidth / 2), (int) h - LABEL_BAR_HEIGHT/2);
            }
        }
    }

    public static void main(String[] args) {
        JFrame f = new JFrame();
        f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        int[] data = {169, 128, 90, 89, 2, 255};
        int[] data2 = {1, 4, 8, 10, 2, 5};
        LineGraph graph = new LineGraph(data, 200);
        f.add(graph);
        f.setSize(400, 400);
        f.setLocation(200, 200);
        f.setVisible(true);
    }

    private void redrawLineGraph() {
        SwingUtilities.invokeLater(
            new Runnable() {
                public void run() {
                    LineGraph.this.repaint();
                }
            });
    }
}
