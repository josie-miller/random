package frc.robot.autons.modes;

import com.choreo.lib.Choreo;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.handoff.Handoff;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.otb_intake.OtbIntake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class ChoreoTest extends SequentialCommandGroup {

    public ChoreoTest(Swerve swerve, Intake intake, OtbIntake otbIntake, Handoff handoff, Shooter shooter){
        addRequirements(swerve, intake, otbIntake, handoff, shooter);
        addCommands(
            new InstantCommand(() -> swerve.setGyroStartingPosition(0)),
            swerve.runChoreoTraj(Choreo.getTrajectory("midCloseForwardMID"), true)
        );
    }

}
