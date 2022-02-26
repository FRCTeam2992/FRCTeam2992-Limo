package frc.lib.Ranging;

public class CargoBallDataPoint implements Comparable<CargoBallDataPoint> {

    // Variables
    private double distance;
    private int mainShooterSpeed;
    private int secondShooterSpeed;
    private double hoodPosition;

    public CargoBallDataPoint(double distance, int mainShooterSpeed, int secondSooterSpeed, double hoodPosition) {
        // Save the Variables
        this.distance = distance;
        this.mainShooterSpeed = mainShooterSpeed;
        this.secondShooterSpeed = secondShooterSpeed;
        this.hoodPosition = hoodPosition;
    }

    public CargoBallDataPoint() {
        this(0.0, 0, 0, 0.0);
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    public double getDistance() {
        return distance;
    }

    public void setMainShooterSpeed(int speed) {
        this.mainShooterSpeed = speed;
    }

    public int getMainShooterSpeed() {
        return mainShooterSpeed;
    }

    public void setSecondShooterSpeed(int speed) {
        this.secondShooterSpeed = speed;
    }

    public int getSecondShooterSpeed() {
        return secondShooterSpeed;
    }

    public void setHoodPosition(double position) {
        this.hoodPosition = position;
    }

    public double getHoodPosition() {
        return hoodPosition;
    }

    @Override
    public int compareTo(CargoBallDataPoint dataPoint) {
        return Double.valueOf(distance).compareTo(dataPoint.getDistance());
    }
}
