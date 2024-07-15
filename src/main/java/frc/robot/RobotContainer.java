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
import frc.robot.autons.AutonomousSelector;
import frc.robot.autons.AutonomousSelector.modes;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.sequences.IntakeSequence;
import frc.robot.commands.sequences.ShootSequence;
import frc.robot.commands.singles.RunHandoff;
import frc.robot.commands.singles.RunIntake;
import frc.robot.commands.singles.RunIntake2;
import frc.robot.commands.singles.SetPivot;
import frc.robot.constants.commandConstants;

public class RobotContainer {
    private final Intake intake = new Intake(new IntakeIOReal());
    private final Handoff handoff = new Handoff(new HandoffIOReal());
    private final Elevator elevator = new Elevator(new ElevatorIOReal());
    private final Shooter shooter = new Shooter(new ShooterIOReal());
    private final OtbIntake otbIntake = new OtbIntake(new OtbIntakeIOReal());
    private final Swerve swerve = new Swerve();
    //public final CommandXboxController controller = new CommandXboxController(1);
    public final CommandXboxController operator = new CommandXboxController(0);
    private AutonomousSelector selector;

    ChoreoTrajectory traj;

    public RobotContainer() {
    swerve.zeroWheels();
    swerve.zeroGyro();
    swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> operator.getRawAxis(XboxController.Axis.kLeftY.value),
                () -> operator.getRawAxis(XboxController.Axis.kLeftX.value), 
                () -> operator.getRawAxis(XboxController.Axis.kRightX.value)
              
            )
        );
    
    configureButtonBindings();
    }

    private void configureButtonBindings() {

        operator.a() //IDLE
            .onTrue(new ParallelCommandGroup(
                new SetPivot(otbIntake, commandConstants.restingDegrees),
                new RunCommand(() -> handoff.runHandoff(0)),
                new RunCommand(() -> intake.runIntake(0)),
                new InstantCommand(() -> elevator.setSetpoint(0)),
                new RunCommand(() -> shooter.setVelocity(0,0))
            ) {{addRequirements(intake, handoff, otbIntake, elevator, shooter);}}
        );

        operator.rightBumper() //INTAKE
            .onTrue(new IntakeSequence(otbIntake, intake, handoff));

        operator.x() //SHOOT SUBWOOFER AMP
            .onTrue(new ShootSequence(intake, handoff, shooter, commandConstants.leftShootTime, commandConstants.leftShootRatio, commandConstants.handoffIntakeVoltage, commandConstants.handoffShooterVoltage, commandConstants.shootLeftVelocity, commandConstants.leftShortTime));

        operator.y() //SHOOT SUBWOOFER MID
            .onTrue(new ShootSequence(intake, handoff, shooter, commandConstants.midShootTime, commandConstants.midShootRatio, commandConstants.handoffIntakeVoltage, commandConstants.handoffShooterVoltage, commandConstants.shootMidVelocity, commandConstants.midShortTime));
       
        operator.b() //SHOOT SUBWOOFER SOURCE
            .onTrue(new ShootSequence(intake, handoff, shooter, commandConstants.rightShootTime, commandConstants.rightShootRatio, commandConstants.handoffIntakeVoltage, commandConstants.handoffShooterVoltage, commandConstants.shootRightVelocity, commandConstants.rightShortTime));
            
        operator.leftBumper() //SHOOT AMP
            .onTrue(new ShootSequence(intake, handoff, shooter, commandConstants.AMPShootTime, commandConstants.AMPShootRatio, commandConstants.handoffIntakeVoltage, commandConstants.handoffShooterVoltage, commandConstants.AMPShooterVelocity, commandConstants.AMPShortTime));

        operator.leftTrigger() //OUTTAKE
            .onTrue(new ParallelCommandGroup(
                new SetPivot(otbIntake, commandConstants.restingDegrees),
                new RunIntake2(intake, commandConstants.outakeVoltage, 2.0),
                new RunHandoff(handoff, -2, 2.0)
            ) {{addRequirements(intake, handoff, otbIntake);}}
        );

        operator.rightTrigger() //INTAKE W/O OTB
            .onTrue(new ParallelCommandGroup(
                new SetPivot(otbIntake, commandConstants.restingDegrees),
                new RunIntake(intake, commandConstants.intakeVoltage)
            ) {{addRequirements(intake, otbIntake);}});

        
        /*controller.b()
            .onTrue(new InstantCommand(() -> swerve.zeroGyro()));*/

    }

    public modes getAutonomousCommand() {
        return selector.get();
    }

    public void configureAutonomousSelector(){
    selector = new AutonomousSelector(swerve, intake, otbIntake, handoff, shooter);
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
