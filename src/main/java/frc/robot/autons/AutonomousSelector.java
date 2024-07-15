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
        TWOPIECEAMP,
        TWOPIECEMID,
        TWOPIECESOURCE,
        PRELOADAMP,
        PRELOADMID,
        PRELOADSOURCE,
        CHOREOTEST
        }
    
    public AutonomousSelector(Swerve swerve, Intake intake, OtbIntake otbIntake, Handoff handoff, Shooter shooter){

        autonomousSelector.setDefaultOption("CHOREOTEST", modes.CHOREOTEST);

        autonomousSelector.addOption("TWOPIECEAMP", modes.TWOPIECEAMP);

        autonomousSelector.addOption("TWOPIECEMID", modes.TWOPIECEMID);

        autonomousSelector.addOption("TWOPIECESOURCE", modes.TWOPIECESOURCE);

        autonomousSelector.addOption("PRELOADAMP", modes.PRELOADAMP);

        autonomousSelector.addOption("PRELOADMID", modes.PRELOADMID);

        autonomousSelector.addOption("PRELOADSOURCE", modes.PRELOADSOURCE);

        SmartDashboard.putData("Auto Choices", autonomousSelector);
    }

    public modes get(){
        return autonomousSelector.getSelected();
    }

}
