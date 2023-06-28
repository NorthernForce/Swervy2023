package frc.robot.robots;

import java.io.IOException;
import java.util.Map;

import org.northernforce.cameras.NFRCamera;
import org.northernforce.cameras.NFRIsaacCamera;
import org.northernforce.gyros.NFRNavX;
import org.northernforce.motors.NFRSparkMax;
import org.northernforce.subsystems.drive.NFRTankDrive;
import org.northernforce.subsystems.drive.NFRTankDrive.NFRTankDriveConfiguration;
import org.northernforce.util.RobotContainer;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class SquishyContainer implements RobotContainer {
    private final NFRTankDrive drive;
    private NFRIsaacCamera realsense;
    private final Field2d field;
    public SquishyContainer()
    {
        NFRTankDriveConfiguration config = new NFRTankDriveConfiguration(
            "drive",
            Units.inchesToMeters(24),
            11.74,
            Units.inchesToMeters(3),
            0.6,
            Units.lbsToKilograms(40),
            Units.feetToMeters(12),
            Units.degreesToRadians(270),
            0,
            1,
            true,
            true,
            false,
            DCMotor.getNeo550(2)
        );
        NFRSparkMax leftSide = new NFRSparkMax(MotorType.kBrushless, 1, 3);
        NFRSparkMax rightSide = new NFRSparkMax(MotorType.kBrushless, 2, 4);
        leftSide.getPIDController().setP(0.2);
        rightSide.getPIDController().setP(0.2);
        leftSide.getPIDController().setI(0.001);
        rightSide.getPIDController().setI(0.001);
        rightSide.setInverted(false);
        leftSide.setInverted(true);
        NFRNavX navx = new NFRNavX();
        drive = new NFRTankDrive(config, leftSide, rightSide, navx);
        field = new Field2d();
        Shuffleboard.getTab("Autonomous").add("Field", field);
        leftSide.getEncoder().setVelocityConversionFactor(leftSide.getEncoder().getVelocityConversionFactor() / 60);
        rightSide.getEncoder().setVelocityConversionFactor(rightSide.getEncoder().getVelocityConversionFactor() / 60);
        try {
            realsense = new NFRIsaacCamera("realsense_cam", AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField(),
                new Transform3d());
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    @Override
    public void bindOI(GenericHID driverHID, GenericHID manipulatorHID)
    {
        if (XboxController.class.isAssignableFrom(driverHID.getClass()) &&
            XboxController.class.isAssignableFrom(manipulatorHID.getClass()))
        {
            XboxController driverController = (XboxController)driverHID;
            drive.setDefaultCommand(drive.getDefaultDriveCommand(
                () -> {
                    return MathUtil.applyDeadband(-driverController.getLeftY(), 0.07);
                },
                () -> {
                    return MathUtil.applyDeadband(-
                    driverController.getRightX(), 0.07);
                }
            ));
            new JoystickButton(driverController, XboxController.Button.kB.value)
                .whileTrue(drive.getStopCommand());
        }
    }
    @Override
    public Map<String, Command> getAutonomousOptions()
    {
        return Map.of("Do nothing", new InstantCommand());
    }
    @Override
    public Map<String, Pose2d> getStartingLocations()
    {
        return Map.of("Unnecessary", new Pose2d());
    }
    @Override
    public void periodic()
    {
        var estimates = ((NFRCamera.ApriltagPipeline)realsense.getCurrentPipeline()).getEstimations();
        for (var estimate : estimates)
        {
            drive.addVisionEstimate(estimate.getFirst(), estimate.getSecond());
        }
        field.setRobotPose(drive.getEstimatedPose());
        SmartDashboard.putNumber("speed", drive.getChassisSpeeds().vxMetersPerSecond);
    }
}
