package org.sert2521.rebuilt2026

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.sert2521.rebuilt2026.subsystems.drivetrain.Drivetrain
import kotlin.jvm.optionals.getOrElse

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 *
 * In Kotlin, it is recommended that all your Subsystems are Kotlin objects. As such, there
 * can only ever be a single instance. This eliminates the need to create reference variables
 * to the various subsystems in this container to pass into to commands. The commands can just
 * directly reference the (single instance of the) object.
 */
object Input {
    private val driverController = CommandXboxController(0)
    private val gunnerController = CommandJoystick(1)

    // TODO: Check and change these
    private val intake = gunnerController.button(0)
    private val reverseIntake = gunnerController.button(1)
    private val revPass = gunnerController.button(2)

    private val manualIndex = gunnerController.button(6)
    private val hoodToStow = gunnerController.button(7)
    private val hoodToPassHalf = gunnerController.button(8)
    private val hoodToPassFull = gunnerController.button(9)

    private val resetRotOffset = driverController.y()
    private val visionAlign = driverController.rightTrigger() // YIPPEEE I love this 2026 change
    private val outtake = driverController.rightBumper()

    private var rotationOffset = Rotation2d.kZero


    init {


        resetRotOffset.onTrue(Commands.runOnce({
            if (DriverStation.getAlliance().getOrElse { DriverStation.Alliance.Blue } == DriverStation.Alliance.Red) {
                Drivetrain.setRotation(Rotation2d.k180deg)
                rotationOffset = Rotation2d.k180deg
            } else {
                Drivetrain.setRotation(Rotation2d.kZero)
                rotationOffset = Rotation2d.kZero
            }
        }))
    }

    fun getLeftX(): Double {
        return -driverController.leftX
    }

    fun getLeftY(): Double {
        return -driverController.leftY
    }

    fun getRightRot(): Double {
        return -driverController.rightX
    }

    fun getRotOffset(): Rotation2d {
        return rotationOffset
    }

    private fun setRumble(amount: Double) {
        driverController.setRumble(GenericHID.RumbleType.kBothRumble, amount)
    }

    fun rumbleBlip(): Command {
        return Commands.runOnce({ setRumble(0.8) }).andThen(Commands.waitSeconds(0.2))
            .andThen(Commands.runOnce({ setRumble(0.0) }))
    }
}