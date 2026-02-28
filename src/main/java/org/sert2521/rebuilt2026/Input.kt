package org.sert2521.rebuilt2026

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.sert2521.rebuilt2026.commands.VisionRotationDrive
import org.sert2521.rebuilt2026.subsystems.Grintake
import org.sert2521.rebuilt2026.subsystems.shooter.HoodedShooter
import org.sert2521.rebuilt2026.subsystems.Indexer
import org.sert2521.rebuilt2026.subsystems.drivetrain.Drivetrain
import kotlin.jvm.optionals.getOrElse

object Input {
    private val driverController = CommandXboxController(0)
    private val gunnerController = CommandJoystick(1)

    private val intake = driverController.rightBumper()
    private val driverOuttake = driverController.rightTrigger()
    private val reverseIntake = driverController.leftBumper()
    private val outtake = gunnerController.button(2)

    private val reverseIndex = gunnerController.button(3)
    private val rev = gunnerController.button(4)

    private val manualIndex = gunnerController.button(1)

    private val visionAlign = driverController.a()


    private val resetRotOffset = driverController.y()
    private val resetRotReal = driverController.x()


    private var rotationOffset = Rotation2d.kZero


    init {

        outtake.whileTrue(Indexer.pulse().unless { !HoodedShooter.isRevved() })
        outtake.onTrue(HoodedShooter.shoot())

        // intake.whileTrue(Indexer.manualIndex())
        intake.whileTrue(Grintake.intake().alongWith(Indexer.index().asProxy()))
        reverseIntake.whileTrue(Grintake.reverse().alongWith(Indexer.reverse().asProxy()))
        rev.onTrue(HoodedShooter.rev())
        reverseIndex.whileTrue(Indexer.reverse())

        manualIndex.whileTrue(Indexer.manualIndex())

        visionAlign.whileTrue(VisionRotationDrive())

        resetRotReal.onTrue(Commands.runOnce({
            if (DriverStation.getAlliance().getOrElse { DriverStation.Alliance.Blue } == DriverStation.Alliance.Red) {
                rotationOffset = Rotation2d.k180deg
                Drivetrain.setRotation(Rotation2d.k180deg)
            } else {
                rotationOffset = Rotation2d.kZero
                Drivetrain.setRotation(Rotation2d.kZero)
            }
        }))

        resetRotOffset.onTrue(Commands.runOnce({
            rotationOffset = Drivetrain.getPose().rotation
        }))
    }
    fun getJoystickInputs(): Triple<Double, Double, Double> {
        return Triple(getLeftX(), getLeftY(), getRightRot())
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
        return Commands.runOnce({ setRumble(0.8) })
            .andThen(Commands.waitSeconds(0.2))
            .andThen(Commands.runOnce({ setRumble(0.0) }))
    }
}