package frc.lib.Ranging;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Handles creating list of setpoints and distances.
 * <p>
 * This class allows you to create list of setpoints and distances for use with
 * shooters. It can find the closest value for either the setpoint or distance.
 */
public class CargoBallInterpolator {

    private List<CargoBallDataPoint> dataPointList;

    public CargoBallInterpolator() {
        dataPointList = new ArrayList<CargoBallDataPoint>();
    }

    /**
     * @param distance the distance value.
     * @param setpoint the desired value for at the distance.
     */
    public void addDataPoint(CargoBallDataPoint dataPoint) {
        dataPointList.add(dataPoint);
        Collections.sort(dataPointList);
    }

    public void removeDataPoint(CargoBallDataPoint dataPoint) {
        dataPointList.remove(dataPoint);
    }

    /**
     * @param distance the distance value.
     * @return the desired value for at the distance.
     */
    public int calcMainShooterSpeed(double distance) {
        int tempMainSpeed = 0;
        System.out.println("calcmainshooterspeed distance " + distance);
        if (dataPointList.size() == 1) {
            tempMainSpeed = dataPointList.get(0).getMainShooterSpeed();
            System.out.println("calcmainshooterspeed list size 1");
        } else if (dataPointList.size() > 1) {
            CargoBallDataPoint upperDataPoint = new CargoBallDataPoint(-1.0, 0, 0, 0.0);
            CargoBallDataPoint lowerDataPoint = new CargoBallDataPoint(-1.0, 0, 0, 0.0);

            for (int i = 0; i < dataPointList.size(); i++) {
                if (dataPointList.get(i).getDistance() >= distance) {
                    upperDataPoint = dataPointList.get(i);

                    break;
                }

                lowerDataPoint = dataPointList.get(i);
            }
            if (lowerDataPoint.getDistance() == -1.0) {
                tempMainSpeed = upperDataPoint.getMainShooterSpeed();
            } else if (upperDataPoint.getDistance() == -1.0) {
                tempMainSpeed = lowerDataPoint.getMainShooterSpeed();
            } else {
                int upperMainSpeed = upperDataPoint.getMainShooterSpeed();
                int lowerMainSpeed = lowerDataPoint.getMainShooterSpeed();

                tempMainSpeed = lerp(lowerMainSpeed, upperMainSpeed, (distance - lowerDataPoint.getDistance())
                        / (upperDataPoint.getDistance() - lowerDataPoint.getDistance()));
            }
        }

        SmartDashboard.putNumber("Main Shooter Speed", tempMainSpeed);
        System.out.println("calcmainshooterspeed returns " + tempMainSpeed);

        return tempMainSpeed;
    }

    public int calcSecondShooterSpeed(double distance) {
        int tempSecondSpeed = 0;

        if (dataPointList.size() == 1) {
            tempSecondSpeed = dataPointList.get(0).getSecondShooterSpeed();
        } else if (dataPointList.size() > 1) {
            CargoBallDataPoint upperDataPoint = new CargoBallDataPoint(-1.0, 0, 0, 0.0);
            CargoBallDataPoint lowerDataPoint = new CargoBallDataPoint(-1.0, 0, 0, 0.0);

            for (int i = 0; i < dataPointList.size(); i++) {
                if (dataPointList.get(i).getDistance() >= distance) {
                    upperDataPoint = dataPointList.get(i);

                    break;
                }

                lowerDataPoint = dataPointList.get(i);
            }
            if (lowerDataPoint.getDistance() == -1.0) {
                tempSecondSpeed = upperDataPoint.getSecondShooterSpeed();
            } else if (upperDataPoint.getDistance() == -1.0) {
                tempSecondSpeed = lowerDataPoint.getSecondShooterSpeed();
            } else {
                int upperSecondSpeed = upperDataPoint.getSecondShooterSpeed();
                int lowerSecondSpeed = lowerDataPoint.getSecondShooterSpeed();

                tempSecondSpeed = lerp(lowerSecondSpeed, upperSecondSpeed, (distance - lowerDataPoint.getDistance())
                        / (upperDataPoint.getDistance() - lowerDataPoint.getDistance()));
            }
        }

        SmartDashboard.putNumber("Second Shooter Speed", tempSecondSpeed);

        return tempSecondSpeed;
    }

    public double calcHoodPosition(double distance) {
        double tempHoodPosition = 0;

        if (dataPointList.size() == 1) {
            tempHoodPosition = dataPointList.get(0).getHoodPosition();
        } else if (dataPointList.size() > 1) {
            CargoBallDataPoint upperDataPoint = new CargoBallDataPoint(-1.0, 0, 0, 0.0);
            CargoBallDataPoint lowerDataPoint = new CargoBallDataPoint(-1.0, 0, 0, 0.0);

            for (int i = 0; i < dataPointList.size(); i++) {
                if (dataPointList.get(i).getDistance() >= distance) {
                    upperDataPoint = dataPointList.get(i);

                    break;
                }

                lowerDataPoint = dataPointList.get(i);
            }
            if (lowerDataPoint.getDistance() == -1.0) {
                tempHoodPosition = upperDataPoint.getHoodPosition();
            } else if (upperDataPoint.getDistance() == -1.0) {
                tempHoodPosition = lowerDataPoint.getHoodPosition();
            } else {
                double upperHoodPosition = upperDataPoint.getHoodPosition();
                double lowerHoodPosition = lowerDataPoint.getHoodPosition();

                tempHoodPosition = lerp(lowerHoodPosition, upperHoodPosition, (distance - lowerDataPoint.getDistance())
                        / (upperDataPoint.getDistance() - lowerDataPoint.getDistance()));
            }
        }

        SmartDashboard.putNumber("Hood Position", tempHoodPosition);

        return tempHoodPosition;
    }

    private double lerp(double start, double end, double count) {
        return start + (count * (end - start));
    }

    private int lerp(int start, int end, double count) {
        return (int) Math.round(start + (count * (end - start)));
    }
}