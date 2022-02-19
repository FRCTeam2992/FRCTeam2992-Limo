// package frc.lib.oi.controller;

// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.GenericHID.Hand;
// import edu.wpi.first.wpilibj2.command.button.Trigger;

// public class TriggerButton extends Trigger {

//     // Variables
//     private XboxController controller;
//     private Hand triggerHand;
//     private double threshhold;

//     public TriggerButton(XboxController controller, Hand triggerHand, double threshhold) {
//         // Variables
//         this.controller = controller;
//         this.triggerHand = triggerHand;
//         this.threshhold = threshhold;
//     }

//     @Override
//     public boolean get() {
//         return Math.abs(controller.getTriggerAxis(triggerHand)) >= threshhold;
//     }
// }
