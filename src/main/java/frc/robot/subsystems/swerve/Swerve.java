package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import frc.robot.constants.canIDConstants;
import frc.robot.constants.swerveConstants;

public class Swerve extends SubsystemBase{
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    public final ModuleIO[] moduleIOs = new ModuleIO[4];
    private final ModuleIOInputsAutoLogged[] moduleInputs = {
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged()
    };
    private Pose2d poseRaw = new Pose2d();
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(swerveConstants.FL, swerveConstants.FR, swerveConstants.BL, swerveConstants.BR);
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getRotation2d(),
        getSwerveModulePositions()); 
    SwerveModuleState setpointModuleStates[] = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
            0,
            0,
            0,
            new Rotation2d()));

    private final SysIdRoutine driveRoutine = new SysIdRoutine(new SysIdRoutine.Config(
        null, 
        Volts.of(3), 
        Seconds.of(4), 
        (state) -> SignalLogger.writeString("state", state.toString())), 
        new SysIdRoutine.Mechanism((
            Measure<Voltage> volts) -> driveVoltage(volts.in(Volts)),
             null, 
             this)
    );

    private final SysIdRoutine steerRoutine = new SysIdRoutine(new SysIdRoutine.Config(
        null, 
        Volts.of(5), 
        Seconds.of(6), 
        (state) -> SignalLogger.writeString("state", state.toString())), 
        new SysIdRoutine.Mechanism((
            Measure<Voltage> volts) -> moduleIOs[0].steerVoltage(volts.in(Volts)),
             null, 
             this)
        );

    public Swerve() {

        gyroIO = new GyroIOReal(canIDConstants.pigeon);

        moduleIOs[0] = new ModuleIOReal(canIDConstants.driveMotor[0], canIDConstants.steerMotor[0], canIDConstants.CANcoder[0],swerveConstants.CANcoderOffsets[0],
        swerveConstants.driveMotorInverts[0], swerveConstants.steerMotorInverts[0], swerveConstants.CANcoderInverts[0]);

        moduleIOs[1] = new ModuleIOReal(canIDConstants.driveMotor[1], canIDConstants.steerMotor[1], canIDConstants.CANcoder[1], swerveConstants.CANcoderOffsets[1],
       swerveConstants.driveMotorInverts[1], swerveConstants.steerMotorInverts[1], swerveConstants.CANcoderInverts[1]);

        moduleIOs[2] = new ModuleIOReal(canIDConstants.driveMotor[2], canIDConstants.steerMotor[2], canIDConstants.CANcoder[2], swerveConstants.CANcoderOffsets[2],
        swerveConstants.driveMotorInverts[2], swerveConstants.steerMotorInverts[2], swerveConstants.CANcoderInverts[2]);

        moduleIOs[3] = new ModuleIOReal(canIDConstants.driveMotor[3], canIDConstants.steerMotor[3], canIDConstants.CANcoder[3], swerveConstants.CANcoderOffsets[3],
        swerveConstants.driveMotorInverts[3], swerveConstants.steerMotorInverts[3], swerveConstants.CANcoderInverts[3]);

        for (int i = 0; i < 4; i++) {
            moduleIOs[i].setDriveBrakeMode(true);
            moduleIOs[i].setTurnBrakeMode(false);
        }
      }


    @Override
    public void periodic(){
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Swerve/Gyro", gyroInputs);
        for (int i = 0; i < 4; i++){
            moduleIOs[i].updateInputs(moduleInputs[i]);
            Logger.processInputs("Swerve/Module/ModuleNum[" + i + "]", moduleInputs[i]);
        }
        
        updateOdometry();
        logModuleStates("SwerveModuleStates/setpointStates", getSetpointStates());
        //logModuleStates("SwerveModuleStates/optimizedSetpointStates", getOptimizedSetPointStates());
        logModuleStates("SwerveModuleStates/MeasuredStates", getMeasuredStates());
        Logger.recordOutput("Odometry/PoseRaw", poseRaw);

    }

    public void requestDesiredState(double x_speed, double y_speed, double rot_speed, boolean fieldRelative, boolean isOpenLoop){

        Rotation2d[] steerPositions = new Rotation2d[4];
        SwerveModuleState[] desiredModuleStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            steerPositions[i] = new Rotation2d(moduleInputs[i].moduleAngleRads);
        }
        Rotation2d gyroPosition = new Rotation2d(gyroInputs.positionRad);
        if (fieldRelative && isOpenLoop){
            desiredModuleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
                x_speed,
                y_speed,
                rot_speed,
                gyroPosition));
            SwerveDriveKinematics.desaturateWheelSpeeds(setpointModuleStates, 12);
            for (int i = 0; i < 4; i++) {
                setpointModuleStates[i] =  SwerveModuleState.optimize(desiredModuleStates[i], steerPositions[i]);
                moduleIOs[i].setDesiredState(setpointModuleStates[i], true);
            }
        }
        else if(fieldRelative && !isOpenLoop){
            desiredModuleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
                x_speed,
                y_speed,
                rot_speed,
                gyroPosition));
            SwerveDriveKinematics.desaturateWheelSpeeds(setpointModuleStates, swerveConstants.maxSpeed);
            for (int i = 0; i < 4; i++) {
                setpointModuleStates[i] =  SwerveModuleState.optimize(desiredModuleStates[i], steerPositions[i]);
                moduleIOs[i].setDesiredState(setpointModuleStates[i], false);
            }
        }
        else if(!fieldRelative && !isOpenLoop){
            desiredModuleStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(
                x_speed,
                y_speed,
                rot_speed
                ));
            SwerveDriveKinematics.desaturateWheelSpeeds(setpointModuleStates, swerveConstants.maxSpeed);
            for (int i = 0; i < 4; i++) {
                setpointModuleStates[i] =  SwerveModuleState.optimize(desiredModuleStates[i], steerPositions[i]);
                moduleIOs[i].setDesiredState(setpointModuleStates[i], false);
            }
        }
        
    }

    public void zeroWheels(){
        for(int i = 0; i < 4; i++){
            moduleIOs[i].resetToAbsolute();
        }
    }

    public void zeroGyro(){
        gyroIO.reset();
    }

    public void updateOdometry(){
        new Rotation2d(gyroInputs.positionRad);
        poseRaw = odometry.update(
                getRotation2d(),
                getSwerveModulePositions());
    }

    public Pose2d getPoseRaw(){
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose){
        odometry.resetPosition(getRotation2d(), getSwerveModulePositions(), pose);
        poseRaw = pose;
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
        ChassisSpeeds desiredChassisSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
        double x_speed = desiredChassisSpeeds.vxMetersPerSecond;
        double y_speed = desiredChassisSpeeds.vyMetersPerSecond;
        double rot_speed = desiredChassisSpeeds.omegaRadiansPerSecond;

        requestDesiredState(x_speed, y_speed, rot_speed, false, false);

    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return kinematics.toChassisSpeeds(getMeasuredStates());
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(gyroInputs.positionRad);
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            modulePositions[i] = new SwerveModulePosition(
                    moduleInputs[i].driveDistanceMeters,
                    new Rotation2d(moduleInputs[i].moduleAngleRads));
        }
        return modulePositions;
    }

    public SwerveModuleState[] getMeasuredStates(){
        SwerveModuleState[] measuredStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++){
            measuredStates[i] = new SwerveModuleState(moduleInputs[i].driveVelocityMetersPerSec, new Rotation2d(moduleInputs[i].moduleAngleRads));
        }
        return measuredStates;
    }

  

    public void driveVoltage(double volts){
        for( int i = 0; i < 4; i++){
            moduleIOs[i].setDriveVoltage(volts);
        }
        
    }

    public Command driveSysIdCmd(){
        return Commands.sequence(
            this.runOnce(() -> SignalLogger.start()),
            driveRoutine
                .quasistatic(Direction.kForward),
                this.runOnce(() -> driveVoltage(0)),
                Commands.waitSeconds(1),
            driveRoutine
                .quasistatic(Direction.kReverse),
                this.runOnce(() -> driveVoltage(0)),
                Commands.waitSeconds(1),  

            driveRoutine
                .dynamic(Direction.kForward),
                this.runOnce(() -> driveVoltage(0)),
                Commands.waitSeconds(1),  

            driveRoutine
                .dynamic(Direction.kReverse),
                this.runOnce(() -> driveVoltage(0)),
                Commands.waitSeconds(1), 
            this.runOnce(() -> SignalLogger.stop())
        );
    }

    public Command steerSysIdCmd(){
        return Commands.sequence(
        this.runOnce(() -> SignalLogger.start()),
            steerRoutine
                .quasistatic(Direction.kForward),
                this.runOnce(() -> moduleIOs[0].steerVoltage(0)),
                Commands.waitSeconds(1),
            steerRoutine
                .quasistatic(Direction.kReverse),
                this.runOnce(() -> moduleIOs[0].steerVoltage(0)),
                Commands.waitSeconds(1),  

            steerRoutine
                .dynamic(Direction.kForward),
                this.runOnce(() -> moduleIOs[0].steerVoltage(0)),
                Commands.waitSeconds(1),  

            steerRoutine
                .dynamic(Direction.kReverse),
                this.runOnce(() -> moduleIOs[0].steerVoltage(0)),
                Commands.waitSeconds(1), 
            this.runOnce(() -> SignalLogger.stop())
        );
    }
    public SwerveModuleState[] getSetpointStates(){
        return setpointModuleStates;
    }

    public double getGyroPositionDegrees(){
        return gyroInputs.positionDegRaw;
    }

    public double getGyroPositionRadians(){
        return gyroInputs.positionRad;
    }

    public double getDriveCurrent(){
        return moduleInputs[0].driveCurrentAmps;
    }

    public void setGyroStartingPosition(double yawDegrees){
        gyroIO.setPosition(yawDegrees);
    }

    private void logModuleStates(String key, SwerveModuleState[] states) {
        List<Double> dataArray = new ArrayList<Double>();
        for (int i = 0; i < 4; i++) {
            dataArray.add(states[i].angle.getRadians());
            dataArray.add(states[i].speedMetersPerSecond);
        }
        Logger.recordOutput(key, dataArray.stream().mapToDouble(Double::doubleValue).toArray());
    }

    public Command runChoreoTraj(ChoreoTrajectory traj) {
        return this.runChoreoTraj(traj, false);
    }

    public Command runChoreoTraj(ChoreoTrajectory traj, boolean resetPose) {
    return choreoFullFollowSwerveCommand(
            traj,
            () -> poseRaw,
            Choreo.choreoSwerveController(
                new PIDController(1, 0.0, 0.0),
                new PIDController(1, 0.0, 0.0),
                new PIDController(0.5, 0.0, 0.0)),
            (ChassisSpeeds speeds) -> this.driveRobotRelative(speeds),
            () -> {
              Optional<Alliance> alliance = DriverStation.getAlliance();
              return alliance.isPresent() && alliance.get() == Alliance.Red;
            },
            this)
        .beforeStarting(
            Commands.runOnce(
                    () -> {
                      if (DriverStation.getAlliance().isPresent()
                          && DriverStation.getAlliance().get().equals(Alliance.Red)) {
                        resetPose(traj.getInitialState().flipped().getPose());
                      } else {
                        resetPose(traj.getInitialPose());
                      }
                    })
                .onlyIf(() -> resetPose));
  }

      public static Command choreoFullFollowSwerveCommand(
      ChoreoTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      ChoreoControlFunction controller,
      Consumer<ChassisSpeeds> outputChassisSpeeds,
      BooleanSupplier mirrorTrajectory,
      Subsystem... requirements) {
        var timer = new Timer();
        return new FunctionalCommand(
        () -> {
          timer.restart();
            Logger.recordOutput(
                "Choreo/Active Traj",
                (mirrorTrajectory.getAsBoolean() ? trajectory.flipped() : trajectory).getPoses());
        },
        () -> {
          Logger.recordOutput(
              "Choreo/Target Pose",
              trajectory.sample(timer.get(), mirrorTrajectory.getAsBoolean()).getPose());
          Logger.recordOutput(
              "Choreo/Target Speeds",
              trajectory.sample(timer.get(), mirrorTrajectory.getAsBoolean()).getChassisSpeeds());
          outputChassisSpeeds.accept(
              controller.apply(
                  poseSupplier.get(),
                  trajectory.sample(timer.get(), mirrorTrajectory.getAsBoolean())));
        },
        (interrupted) -> {
          timer.stop();
          outputChassisSpeeds.accept(new ChassisSpeeds());
        },
        () -> {
          var finalPose =
              mirrorTrajectory.getAsBoolean()
                  ? trajectory.getFinalState().flipped().getPose()
                  : trajectory.getFinalState().getPose();
          Logger.recordOutput("Swerve/Current Traj End Pose", finalPose);
          return timer.hasElapsed(trajectory.getTotalTime())
              && (MathUtil.isNear(finalPose.getX(), poseSupplier.get().getX(), 0.4)
                  && MathUtil.isNear(finalPose.getY(), poseSupplier.get().getY(), 0.4)
                  && Math.abs(
                          (poseSupplier.get().getRotation().getDegrees()
                                  - finalPose.getRotation().getDegrees())
                              % 360)
                      < 20.0);
        },
        requirements);
  }

}