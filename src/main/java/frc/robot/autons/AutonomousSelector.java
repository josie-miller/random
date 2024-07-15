package frc.robot.autons;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.otb_intake.OtbIntake;
import frc.robot.subsystems.handoff.Handoff;
import frc.robot.subsystems.shooter.Shooter;

public class AutonomousSelector {
    private SendableChooser<modes> autonomousSelector = new SendableChooser<modes>();
    String mode;
     public enum modes{
        TWOPIECEAMP
        }
    
    public AutonomousSelector(Swerve swerve, Intake intake, OtbIntake otbIntake, Handoff handoff, Shooter shooter){

        autonomousSelector.setDefaultOption("TWOPIECEAMP", modes.TWOPIECEAMP);

        SmartDashboard.putData("Auto Choices", autonomousSelector);
    }

    public modes get(){
        return autonomousSelector.getSelected();
    }

}
