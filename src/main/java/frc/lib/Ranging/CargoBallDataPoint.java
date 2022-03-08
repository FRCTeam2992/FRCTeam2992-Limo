package frc.lib.Ranging;

public class CargoBallDataPoint implements Comparable<CargoBallDataPoint> {

    // Variables
    private double distance;
    private double mainShooterSpeed;
    private double secondShooterSpeed;
    private double hoodPosition;

    public CargoBallDataPoint(double distance, double mainShooterSpeed, double secondShooterSpeed, double hoodPosition) {
        // Save the Variables
        this.distance = distance;
        this.mainShooterSpeed = mainShooterSpeed;
        this.secondShooterSpeed = secondShooterSpeed;
        this.hoodPosition = hoodPosition;
    }

    public CargoBallDataPoint() {
        this(0.0, 0.0, 0.0, 0.0);
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

    public double getMainShooterSpeed() {
        return mainShooterSpeed;
    }

    public void setSecondShooterSpeed(int speed) {
        this.secondShooterSpeed = speed;
    }

    public double getSecondShooterSpeed() {
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
