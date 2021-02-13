package frc.team3324.robot

import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil
import io.github.oblarg.oblog.Logger

class Robot: TimedRobot() {

    private val compressor = Compressor()
    private val ultrasonic = AnalogInput(1)
    private val depRangeInches: Double
        get() = ultrasonic.value.toDouble() * 0.125

    companion object {
        val light = DigitalOutput(1)
        val robotContainer = RobotContainer()
        val pdp = PowerDistributionPanel()
    }

    override fun robotInit() {
        SmartDashboard.putNumber("Volt", 0.0)
        CameraServer.getInstance().startAutomaticCapture()
        LiveWindow.disableAllTelemetry()
        compressor.start()
    }

    fun enabledInit() {
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
        Logger.updateEntries()
        Logger.updateEntries()
    }

    override fun teleopInit() {
        enabledInit()
    }

    override fun teleopPeriodic() {
    }

    override fun autonomousInit() {
        val trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/StraightLine.wpilib.json")
        val trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath)
        if (trajectory != null) {
            System.out.println("beep boop i am here <-----------------------------------------------------")
            robotContainer.getAutoCommand(trajectory).schedule()
            System.out.println("meep moop i am gone <-----------------------------------------------------")
        }
    }
    override fun autonomousPeriodic() {
    }
}
