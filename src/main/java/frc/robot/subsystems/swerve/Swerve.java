package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
    private final SwerveDriveKinematics swerveKinematics;
    private final SwerveDriveOdometry swerveOdometry;
    private final Module[] modules;
    private final Gyro gyro;

    public Swerve() {

        swerveKinematics = new SwerveDriveKinematics(
            Constants.swerveConstants.FL,
            Constants.swerveConstants.FR,
            Constants.swerveConstants.BL,
            Constants.swerveConstants.BR
        );

        gyro = new Gyro();
        modules = new Module[]{
            new Module(0),
            new Module(1),
            new Module(2),
            new Module(3)
        };

        swerveOdometry = new SwerveDriveOdometry(swerveKinematics, getGyroYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        ChassisSpeeds chassisSpeeds = fieldRelative ?
            ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), 
                translation.getY(), 
                rotation, 
                getHeading()
            ) :
            new ChassisSpeeds(
                translation.getX(), 
                translation.getY(), 
                rotation
            );

        SwerveModuleState[] swerveModuleStates = swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.swerveConstants.maxSpeed);

        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(swerveModuleStates[i], isOpenLoop);
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.swerveConstants.maxSpeed);
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(desiredStates[i], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return gyro.getYaw();
    }

    public void resetModulesToAbsolute() {
        for (Module mod : modules) {
            mod.resetToAbsolute();
        }
    }

    public void requestDesiredState(double x_speed, double y_speed, double rot_speed, boolean fieldRelative, boolean isOpenLoop) {
        Rotation2d[] steerPositions = new Rotation2d[modules.length];
        SwerveModuleState[] desiredModuleStates = new SwerveModuleState[modules.length];
        SwerveModuleState[] setpointModuleStates = new SwerveModuleState[modules.length];

        //get current module angles
        for (int i = 0; i < modules.length; i++) {
            steerPositions[i] = modules[i].getPosition().angle;
        }

        //get gyro heading
        Rotation2d gyroPosition = getGyroYaw();

        //determine the desired module states based on the mode
        if (fieldRelative && isOpenLoop) {
            desiredModuleStates = swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
                x_speed,
                y_speed,
                rot_speed,
                gyroPosition));
        } else if (fieldRelative && !isOpenLoop) {
            desiredModuleStates = swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
                x_speed,
                y_speed,
                rot_speed,
                gyroPosition));
        } else if (!fieldRelative && !isOpenLoop) {
            desiredModuleStates = swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(
                x_speed,
                y_speed,
                rot_speed));
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, Constants.swerveConstants.maxSpeed);
        for (int i = 0; i < modules.length; i++) {
            setpointModuleStates[i] = SwerveModuleState.optimize(desiredModuleStates[i], steerPositions[i]);
            modules[i].setDesiredState(setpointModuleStates[i], isOpenLoop);
        }
    }

    public void zeroWheels() {
        for (Module module : modules) {
            module.resetToAbsolute();
        }
    }
    
    public void zeroGyro() {
        gyro.reset();
    }    

    @Override
    public void periodic() {
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        for (int i = 0; i < modules.length; i++) {
            SmartDashboard.putNumber("Mod " + i + " CANcoder", modules[i].getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + i + " Angle", modules[i].getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + i + " Velocity", modules[i].getState().speedMetersPerSecond);
        }
    }
}
