package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;


import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.swerveConstants;
import frc.robot.subsystems.swerve.GyroIO.GyroIOInputs;
import frc.robot.subsystems.swerve.ModuleIO.ModuleIOInputs;


public class Swerve extends SubsystemBase{
    private final GyroIO gyroIO = new GyroIOPigeon(canIDConstants.pigeon);
    private final GyroIOInputs gyroInputs = new GyroIOInputs();
    public final ModuleIO[] moduleIOs = new ModuleIO[4];
    private final ModuleIOInputs[] moduleInputs = {
            new ModuleIOInputs(),
            new ModuleIOInputs(),
            new ModuleIOInputs(),
            new ModuleIOInputs()
    };
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(swerveConstants.FL, swerveConstants.FR, swerveConstants.BL,
        swerveConstants.BR);
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

    private final SysIdRoutine steerRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, 
            Volts.of(5), 
            Seconds.of(6), 
            (state) -> SignalLogger.writeString("state", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> moduleIOs[0].steerVoltage(volts.in(Volts)),
            null, 
            this
        )
    );
    
    

    public Swerve() {

        moduleIOs[0] = new ModuleIOTalonFX(canIDConstants.driveMotor[0], canIDConstants.steerMotor[0], canIDConstants.CANcoder[0],swerveConstants.CANcoderOffsets[0],
        swerveConstants.driveMotorInverts[0], swerveConstants.steerMotorInverts[0], swerveConstants.CANcoderInverts[0]);

        moduleIOs[1] = new ModuleIOTalonFX(canIDConstants.driveMotor[1], canIDConstants.steerMotor[1], canIDConstants.CANcoder[1], swerveConstants.CANcoderOffsets[1],
       swerveConstants.driveMotorInverts[1], swerveConstants.steerMotorInverts[1], swerveConstants.CANcoderInverts[1]);

        moduleIOs[2] = new ModuleIOTalonFX(canIDConstants.driveMotor[2], canIDConstants.steerMotor[2], canIDConstants.CANcoder[2], swerveConstants.CANcoderOffsets[2],
        swerveConstants.driveMotorInverts[2], swerveConstants.steerMotorInverts[2], swerveConstants.CANcoderInverts[2]);

        moduleIOs[3] = new ModuleIOTalonFX(canIDConstants.driveMotor[3], canIDConstants.steerMotor[3], canIDConstants.CANcoder[3], swerveConstants.CANcoderOffsets[3],
        swerveConstants.driveMotorInverts[3], swerveConstants.steerMotorInverts[3], swerveConstants.CANcoderInverts[3]);

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
        odometry.update(
                getRotation2d(),
                getSwerveModulePositions());
    }

    public Pose2d getPoseRaw(){
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose){
        odometry.resetPosition(getRotation2d(), getSwerveModulePositions(), pose);
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

 
}