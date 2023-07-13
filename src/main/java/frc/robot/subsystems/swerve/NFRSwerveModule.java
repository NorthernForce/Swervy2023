package frc.robot.subsystems.swerve;

import java.util.Optional;

import org.northernforce.encoders.NFREncoder;
import org.northernforce.motors.NFRMotorController;
import org.northernforce.util.NFRFeedbackProvider;
import org.northernforce.util.NFRInternalPositionalPIDFeedback;
import org.northernforce.util.NFRInternalVelocityPIDFeedback;
import org.northernforce.util.NFRLinearFeedback;
import org.northernforce.util.NFRSimplePIDFeedback;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class NFRSwerveModule
{
    protected final NFRMotorController driveMotor, turnMotor;
    protected final Optional<NFREncoder> driveEncoder, turnEncoder;
    protected NFRFeedbackProvider driveFeedback, turnFeedback;
    public NFRSwerveModule(NFRMotorController driveMotor, Optional<NFREncoder> driveEncoder,
        NFRMotorController turnMotor, Optional<NFREncoder> turnEncoder)
    {
        this.driveMotor = driveMotor;
        this.driveEncoder = driveEncoder;
        this.turnMotor = turnMotor;
        this.turnEncoder = turnEncoder;
        this.driveFeedback = null;
        this.turnFeedback = null;
    }
    public void setFeedback(NFRFeedbackProvider driveFeedback, NFRFeedbackProvider turnFeedback)
    {
        this.driveFeedback = driveFeedback;
        this.turnFeedback = turnFeedback;
    }
    public double getDistance()
    {
        if (driveEncoder.isPresent())
        {
            return driveEncoder.get().getPosition();
        }
        else
        {
            return driveMotor.getSelectedEncoder().getPosition();
        }
    }
    public double getVelocity()
    {
        if (driveEncoder.isPresent())
        {
            return driveEncoder.get().getVelocity();
        }
        else
        {
            return driveMotor.getSelectedEncoder().getVelocity();
        }
    }
    public Rotation2d getAngle()
    {
        if (turnEncoder.isPresent())
        {
            return Rotation2d.fromRotations(turnEncoder.get().getPosition());
        }
        else
        {
            return Rotation2d.fromRotations(turnMotor.getSelectedEncoder().getPosition());
        }
    }
    public SwerveModulePosition getPosition()
    {
        return new SwerveModulePosition(getDistance(), getAngle());
    }
    public SwerveModuleState getState()
    {
        return new SwerveModuleState(getVelocity(), getAngle());
    }
    public void setState(NFRFeedbackProvider driveFeedback, NFRFeedbackProvider turnFeedback, SwerveModuleState state)
    {
        state = SwerveModuleState.optimize(state, getAngle());
        this.driveFeedback = driveFeedback;
        this.turnFeedback = turnFeedback;
        driveFeedback.setSetpoint(state.speedMetersPerSecond);
        turnFeedback.setSetpoint(state.angle.getRotations());
    }
    public void setState(SwerveModuleState state)
    {
        state = SwerveModuleState.optimize(state, getAngle());
        if (driveFeedback != null)
        {
            driveFeedback.setSetpoint(state.speedMetersPerSecond);
        }
        else
        {
            driveMotor.set(state.speedMetersPerSecond);
        }
        if (turnFeedback != null)
        {
            turnFeedback.setSetpoint(state.angle.getRotations());
        }
        else
        {
            turnMotor.setPosition(0, state.angle.getRotations());
        }
    }
    public void computeFeedback()
    {
        if (driveFeedback != null)
        {
            driveFeedback.runFeedback(getVelocity());
        }
        if (turnFeedback != null)
        {
            turnFeedback.runFeedback(getAngle().getRotations());
        }
    }
    public static NFRSwerveModule createSimplePIDSwerve(NFRMotorController driveMotor, Optional<NFREncoder> driveEncoder,
        NFRMotorController turnMotor, Optional<NFREncoder> turnEncoder, PIDController turnController)
    {
        NFRSwerveModule module = new NFRSwerveModule(driveMotor, driveEncoder, turnMotor, turnEncoder);
        module.setFeedback(new NFRLinearFeedback(1, driveMotor::set),
            new NFRSimplePIDFeedback(turnMotor::set, turnController));
        return module;
    }
    public static NFRSwerveModule createIntegratedPIDSwerve(NFRMotorController driveMotor, Optional<NFREncoder> driveEncoder,
        NFRMotorController turnMotor, Optional<NFREncoder> turnEncoder, int pidSlot, double tolerance)
    {
        NFRSwerveModule module = new NFRSwerveModule(driveMotor, driveEncoder, turnMotor, turnEncoder);
        module.setFeedback(new NFRLinearFeedback(1, driveMotor::set),
            new NFRInternalPositionalPIDFeedback(turnMotor, pidSlot, false, tolerance));
        return module;
    }
    public static NFRSwerveModule createIntegratedTrapezoidalSwerve(NFRMotorController driveMotor,
        Optional<NFREncoder> driveEncoder, NFRMotorController turnMotor, Optional<NFREncoder> turnEncoder,
        int pidSlot, double tolerance)
    {
        NFRSwerveModule module = new NFRSwerveModule(driveMotor, driveEncoder, turnMotor, turnEncoder);
        module.setFeedback(new NFRLinearFeedback(1, driveMotor::set),
            new NFRInternalPositionalPIDFeedback(turnMotor, pidSlot, true, tolerance));
        return module;
    }
    public static NFRSwerveModule createIntegratedPIDSwerveWithVelocity(NFRMotorController driveMotor,
        Optional<NFREncoder> driveEncoder, NFRMotorController turnMotor, Optional<NFREncoder> turnEncoder,
        int positionalPidSlot, int velocityPidSlot, double positionalTolernance, double velocityTolerance)
    {
        NFRSwerveModule module = new NFRSwerveModule(driveMotor, driveEncoder, turnMotor, turnEncoder);
        module.setFeedback(new NFRInternalVelocityPIDFeedback(driveMotor, velocityPidSlot, velocityTolerance),
            new NFRInternalPositionalPIDFeedback(turnMotor, positionalPidSlot, false,
                positionalTolernance));
        return module;
    }
    public static NFRSwerveModule createIntegratedTrapezoidalSwerveWithVelocity(NFRMotorController driveMotor,
        Optional<NFREncoder> driveEncoder, NFRMotorController turnMotor, Optional<NFREncoder> turnEncoder,
        int positionalPidSlot, int velocityPidSlot, double positionalTolernance, double velocityTolerance)
    {
        NFRSwerveModule module = new NFRSwerveModule(driveMotor, driveEncoder, turnMotor, turnEncoder);
        module.setFeedback(new NFRInternalVelocityPIDFeedback(driveMotor, velocityPidSlot, velocityTolerance),
            new NFRInternalPositionalPIDFeedback(turnMotor, positionalPidSlot, false,
                positionalTolernance));
        return module;
    }
}