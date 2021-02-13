package frc.team3324.robot

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.XboxController.Button
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.RamseteController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PIDCommand
import edu.wpi.first.wpilibj2.command.RamseteCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.team3324.library.commands.MotorCommand
import frc.team3324.robot.drivetrain.DriveTrain
import frc.team3324.robot.drivetrain.commands.teleop.Drive
import frc.team3324.robot.drivetrain.commands.teleop.GyroTurn
import frc.team3324.robot.util.Camera
import frc.team3324.robot.util.Consts
import frc.team3324.library.commands.ToggleLightCommand
import frc.team3324.library.subsystems.MotorSubsystem
import frc.team3324.robot.drivetrain.commands.auto.MeterForward
import io.github.oblarg.oblog.Logger

class RobotContainer {
    private val driveTrain = DriveTrain()


    private val table = NetworkTableInstance.getDefault()

    private val primaryController = XboxController(0)
    private val secondaryController = XboxController(1)

    private val primaryRightX: Double
        get() = primaryController.getX(GenericHID.Hand.kLeft)
    private val primaryLeftY: Double
        get() = primaryController.getY(GenericHID.Hand.kRight)

    private val primaryTriggerRight: Double
        get() = primaryController.getTriggerAxis(GenericHID.Hand.kRight)
    private val primaryTriggerLeft: Double
        get() = primaryController.getTriggerAxis(GenericHID.Hand.kLeft)

    private val secondaryRightX: Double
        get() = secondaryController.getX(GenericHID.Hand.kLeft)
    private val secondRightY: Double
        get() = secondaryController.getY(GenericHID.Hand.kRight)
    private val secondLeftY: Double
        get() = secondaryController.getY(GenericHID.Hand.kLeft)

    private val secondTriggerRight: Double
        get() = secondaryController.getTriggerAxis(GenericHID.Hand.kRight)
    private val secondTriggerLeft: Double
        get() = secondaryController.getTriggerAxis(GenericHID.Hand.kLeft)




   init {
       Robot.light.set(true)
       Logger.configureLoggingAndConfig(this, true)
       driveTrain.defaultCommand = Drive(driveTrain, {primaryController.getY(GenericHID.Hand.kLeft)}, {primaryController.getX(GenericHID.Hand.kRight)})

       configureButtonBindings()

   }

    private fun configureButtonBindings() {
        JoystickButton(primaryController, Button.kA.value).whenPressed(MeterForward(driveTrain, TrapezoidProfile.State(6.0, 0.0)))
        /*JoystickButton(primaryController, Button.kY.value).whileHeld(GyroTurn(
                driveTrain,
                1.0/70.0,
                (Consts.DriveTrain.ksVolts + 0.3)/12,
                {cameraTable.getEntry("targetYaw").getDouble(0.0)},
                   {input -> driveTrain.curvatureDrive(0.0, input, true)}
        ))*/

    }


    fun rumbleController(rumbleLevel: Double) {

        secondaryController.setRumble(GenericHID.RumbleType.kRightRumble, rumbleLevel)
    }

    fun getAutoCommand(trajectory: Trajectory): Command {
        val autoVoltageConstraint = DifferentialDriveVoltageConstraint(
            SimpleMotorFeedforward(Consts.DriveTrain.ksVolts,
                Consts.DriveTrain.LOW_GEAR_KV,
                Consts.DriveTrain.LOW_GEAR_KA),
            Consts.DriveTrain.kDriveKinematics, 10.0) // maxVoltage of 10 allows for head room

        driveTrain.resetOdometry(trajectory.initialPose)

        val ramseteCommand = RamseteCommand(
            trajectory,
            {driveTrain.pose},
            RamseteController(Consts.DriveTrain.kRamseteB, Consts.DriveTrain.kRamseteZeta),
            SimpleMotorFeedforward(Consts.DriveTrain.ksVolts, Consts.DriveTrain.LOW_GEAR_KV, Consts.DriveTrain.LOW_GEAR_KA),
            Consts.DriveTrain.kDriveKinematics,
            {driveTrain.wheelSpeeds},
            PIDController(Consts.DriveTrain.kP, 0.0, 0.0),
            PIDController(Consts.DriveTrain.kP, 0.0, 0.0),
            driveTrain::tankDriveVolts,
            driveTrain
        )

        driveTrain.resetOdometry(trajectory.initialPose)

        return ramseteCommand.andThen({driveTrain.tankDriveVolts(0.0, 0.0)}, driveTrain)
    }
}