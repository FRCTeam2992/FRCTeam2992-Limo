package frc.lib.oi.controller;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerButton extends Trigger {

    // Variables
    private XboxController controller;
    private double threshhold;
    private String hand;

    public TriggerButton(XboxController controller, double threshhold, String hand) {
        // Variables
        this.controller = controller;
        this.threshhold = threshhold;
        this.hand = hand;
    }

    @Override
    public boolean get() {
        if (hand == "left") {
            return Math.abs(controller.getLeftTriggerAxis()) >= threshhold;
        } else {
            return Math.abs(controller.getRightTriggerAxis()) >= threshhold;
        }
    }

}
