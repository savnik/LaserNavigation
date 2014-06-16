/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package marg.model;

/**
 *
 * @author Daniel
 */
public class ValueLimit {

    public final static int LESS_THAN_EQUALS = 1;
    public final static int LESS_THAN = 2;
    private int minimumComparison;
    private int maximumComparison;
    private double maxValue;
    private double minValue;

    public ValueLimit(double minValue, int minComparison, double maxValue, int maxComparison) {
        this.maxValue = maxValue;
        this.minValue = minValue;
        minimumComparison = minComparison;
        maximumComparison = maxComparison;
    }

    public boolean checkCorrect(double value) {
        boolean minCorrect = false;
        boolean maxCorrect = false;
        if (minimumComparison == LESS_THAN_EQUALS) {
            minCorrect = minValue <= value;
        } else {
            minCorrect = minValue < value;
        }
        if (maximumComparison == LESS_THAN_EQUALS) {
            maxCorrect = maxValue >= value;
        } else {
            maxCorrect = maxValue > value;
        }
        boolean isCorrect = (minCorrect && maxCorrect);
        return isCorrect;
    }

    public double getMaxValue() {
        return maxValue;
    }

    public double getMinValue() {
        return minValue;
    }

    public int getMaximumComparison() {
        return maximumComparison;
    }

    public int getMinimumComparison() {
        return minimumComparison;
    }

    public String toString() {
        return "ValueLimit (min "+ minValue +", max "+ maxValue +", MinLTE "+ (minimumComparison == LESS_THAN_EQUALS) +", MaxLTE "+ (maximumComparison == LESS_THAN_EQUALS) +")";
    }
}
