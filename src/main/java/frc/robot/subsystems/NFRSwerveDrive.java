package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.northernforce.gyros.NFRGyro;
import org.northernforce.subsystems.drive.NFRDrive;
import org.northernforce.util.NFRFeedbackProvider;

import frc.robot.subsystems.swerve.NFRSwerveModule;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

public class NFRSwerveDrive extends NFRDrive
{
    public static class NFRSwerveDriveConfiguration extends NFRDriveConfiguration
    {
        protected Translation2d[] moduleTranslations;
        public NFRSwerveDriveConfiguration(String name)
        {
            super(name);
        }
        public NFRSwerveDriveConfiguration(String name, Translation2d[] moduleTranslations)
        {
            super(name);
            this.moduleTranslations = moduleTranslations;
        }
        public NFRSwerveDriveConfiguration withModuleTranslations(Translation2d[] moduleTranslations)
        {
            this.moduleTranslations = moduleTranslations;
            return this;
        }
    }
    protected final NFRGyro gyro;
    protected final SwerveDriveKinematics kinematics;
    protected final SwerveDrivePoseEstimator poseEstimator;
    protected final NFRSwerveModule[] modules;
    public NFRSwerveDrive(NFRSwerveDriveConfiguration config, NFRGyro gyro, NFRSwerveModule... modules)
    {
        super(config);
        this.gyro = gyro;
        kinematics = new SwerveDriveKinematics(config.moduleTranslations);
        this.modules = modules;
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyro.getGyroYaw(), getModulePositions(), new Pose2d());
    }
    public SwerveModulePosition[] getModulePositions()
    {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < positions.length; i++)
        {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }
    public SwerveModuleState[] getModuleStates()
    {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < states.length; i++)
        {
            states[i] = modules[i].getState();
        }
        return states;
    }
    @Override
    public Pose2d getEstimatedPose()
    {
        return poseEstimator.getEstimatedPosition();
    }
    @Override
    public void resetPose(Pose2d pose)
    {
        poseEstimator.resetPosition(gyro.getGyroYaw(), getModulePositions(), pose);
    }
    @Override
    public void addVisionEstimate(double timestamp, Pose2d pose)
    {
        poseEstimator.addVisionMeasurement(pose, timestamp);
    }
    @Override
    public ChassisSpeeds getChassisSpeeds()
    {
        return kinematics.toChassisSpeeds(getModuleStates());
    }
    public void setChassisSpeeds(ChassisSpeeds speeds)
    {
        SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(speeds);
        for (int i = 0; i < targetStates.length; i++)
        {
            modules[i].setState(targetStates[i]);
        }
    }
    public void setChassisSpeeds(NFRFeedbackProvider[] driveFeedback, NFRFeedbackProvider[] turnFeedback,
        ChassisSpeeds speeds)
    {
        SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(speeds);
        for (int i = 0; i < targetStates.length; i++)
        {
            modules[i].setState(driveFeedback[i], turnFeedback[i], targetStates[i]);
        }
    }
    public Command getDefaultDriveCommand(DoubleSupplier... suppliers)
    {
        return null;
    }
    public Command getStopCommand()
    {
        return null;
    }
    public Command getDriveMetersCommand(double meters)
    {
        return null;
    }
}