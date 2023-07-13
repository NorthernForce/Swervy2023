package frc.robot.robots;

import java.util.Map;

import org.northernforce.util.NFRRobotContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;

public class SwervyContainer implements NFRRobotContainer
{
    public SwervyContainer()
    {
    }
    @Override
    public void bindOI(GenericHID driverHID, GenericHID manipulatorHID)
    {
    }
    @Override
    public Map<String, Command> getAutonomousOptions()
    {
        return null;
    }
    @Override
    public Map<String, Pose2d> getStartingLocations()
    {
        return null;
    }
}
