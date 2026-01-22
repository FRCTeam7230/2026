package frc.robot.utils;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonMappings {
    public static Trigger button(XboxController controller, int buttonNumber){

        //xboxcontoller
        if(buttonNumber == -2){
            return new Trigger(() -> controller.getRawAxis(/*change*/2) > 0.5);
        }
        if(buttonNumber == -3){
            return new Trigger(() -> controller.getRawAxis(/*change*/3) > 0.5);
        }
        return buttonNumber%45==0 ? new POVButton(controller, buttonNumber) : new JoystickButton(controller, buttonNumber);
    }
}