package frc.robot;

import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.handoff.Handoff;
import frc.robot.subsystems.handoff.HandoffIOReal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.otb_intake.OtbIntake;
import frc.robot.subsystems.otb_intake.OtbIntakeIOReal;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.constants.commandConstants;

public class RobotContainer {
    private final Intake intake = new Intake(new IntakeIOReal());
    private final Handoff handoff = new Handoff(new HandoffIOReal());
    private final Elevator elevator = new Elevator(new ElevatorIOReal());
    private final Shooter shooter = new Shooter(new ShooterIOReal());
    private final OtbIntake otbIntake = new OtbIntake(new OtbIntakeIOReal());
    private final Swerve swerve = new Swerve();
    public final CommandXboxController operator = new CommandXboxController(0);

    ChoreoTrajectory traj;

    public RobotContainer() {
    swerve.zeroWheels();
    swerve.zeroGyro();
    
    configureButtonBindings();
    }

    private void configureButtonBindings() {

        operator.a() 
            .onTrue(elevator.runSysIdCmd());

        operator.b()
            .onTrue(otbIntake.runSysIdCmd());

        operator.x()
            .onTrue(shooter.shooterSysIdCmd());

        operator.rightTrigger()
            .onTrue(swerve.driveSysIdCmd());

        operator.leftTrigger()
            .onTrue(swerve.steerSysIdCmd());

    }

    public void getAutonomousCommand() {
    }

    public void configureAutonomousSelector(){
    }

    public Swerve getSwerve(){
        return swerve;
    }

    public Intake getIntake(){
        return intake;
    }

    public OtbIntake getOtbIntake(){
        return otbIntake;
    }

    public Handoff getHandoff(){
        return handoff;
    }

    public Shooter getShooter(){
        return shooter;
    }


}
