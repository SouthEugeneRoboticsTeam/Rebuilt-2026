package org.sert2521.rebuilt2026

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.sert2521.rebuilt2026.commands.HoodedShooterCommands
import org.sert2521.rebuilt2026.commands.JoystickDriveLineAssist
import org.sert2521.rebuilt2026.commands.JoystickDriveRotationAlign
import org.sert2521.rebuilt2026.subsystems.Indexer
import org.sert2521.rebuilt2026.subsystems.Intake
import org.sert2521.rebuilt2026.subsystems.Wrist
import org.sert2521.rebuilt2026.subsystems.drivetrain.Drivetrain
import org.sert2521.rebuilt2026.subsystems.hooded_shooter.Flywheel
import org.sert2521.rebuilt2026.subsystems.hooded_shooter.Hood
import org.sert2521.rebuilt2026.util.ZoneUtil
import java.util.function.Supplier
import kotlin.jvm.optionals.getOrElse

object Input {
    private val driverController = CommandXboxController(0)
    private val gunnerController = CommandJoystick(1)

    /* Driver */
    private val intake = driverController.rightTrigger()
    private val wristDown = driverController.leftTrigger()
    private val reverseIntake = driverController.leftBumper()
    private val robotOriented = driverController.rightBumper()

    private val scoringAlign = driverController.x()
    private val utilAlign = driverController.b()
    private val lineAssist = driverController.a()

    private val resetRotOffset = driverController.y()
    private val resetRotReal = driverController.start()

    /* Gunner */
    private val outtake = gunnerController.button(2)
    private val stopRev = gunnerController.button(8)

    private val manualIndex = gunnerController.button(1)
    private val reverseIndex = gunnerController.button(3)

    private val revHub = gunnerController.button(4)
    private val revPass = gunnerController.button(9)
    private val revFull = gunnerController.button(10)

    // Debugging, on gunner
    private val hoodUp = gunnerController.button(7)
    private val hoodDown = gunnerController.button(6)

    private val increaseFlywheel = gunnerController.button(11)
    private val decreaseFlywheel = gunnerController.button(16)

    private val startFlywheelLiveTuning = gunnerController.button(5).multiPress(3, 2.0)

    private var rotationOffset = Rotation2d.kZero
    private var passing = false


    init {
        outtake.whileTrue(Indexer.pulse().alongWith(runOnce({ Flywheel.startTimer() })).unless { !Flywheel.isRevved() })
        outtake.onFalse(runOnce({ Flywheel.stopTimer() }))
        stopRev.onTrue(HoodedShooterCommands.stop())

        hoodDown.onTrue(Hood.setPosition { ShooterConstants.hoodMin })
        hoodUp.onTrue(Hood.setPosition { ShooterConstants.hoodMax })

        intake.whileTrue(Intake.intake().alongWith(Indexer.manualIndex().asProxy()))
        wristDown.whileTrue(Wrist.downSafeDepot())
        wristDown.onFalse(Wrist.up())

        reverseIntake.whileTrue(Intake.reverse().alongWith(Indexer.reverse().asProxy()))
        revHub.onTrue(HoodedShooterCommands.revAndTrackHub().alongWith(runOnce({ passing = false })))
        revPass.onTrue(HoodedShooterCommands.revAndTrackPass().alongWith(runOnce({ passing = true })))
        revFull.onTrue(HoodedShooterCommands.revFarPass().alongWith(runOnce({ passing = true })))

        reverseIndex.whileTrue(Indexer.reverse())

        manualIndex.whileTrue(Indexer.manualIndex())

        scoringAlign.whileTrue(JoystickDriveRotationAlign(::getFieldOriented, ZoneUtil::getScoringRotationTarget))
        utilAlign.whileTrue(JoystickDriveRotationAlign(::getFieldOriented, ZoneUtil::getUtilRotationTarget))
        lineAssist.whileTrue(JoystickDriveLineAssist({ ZoneUtil.getDriverAssistLine().`in`(Meters) }, ZoneUtil::getDriverAssistRotation))

        increaseFlywheel.onTrue(runOnce({ OtherConstsants.flywheelLiveSetpoint += RPM.of(10.0) }))
        decreaseFlywheel.onTrue(runOnce({ OtherConstsants.flywheelLiveSetpoint -= RPM.of(10.0) }))

        startFlywheelLiveTuning.onTrue(HoodedShooterCommands.liveTuning())

        resetRotReal.onTrue(runOnce({
            if (DriverStation.getAlliance().getOrElse { DriverStation.Alliance.Blue } == DriverStation.Alliance.Red) {
                rotationOffset = Rotation2d.k180deg
                Drivetrain.setRotation(Rotation2d.k180deg)
            } else {
                rotationOffset = Rotation2d.kZero
                Drivetrain.setRotation(Rotation2d.kZero)
            }
        }))

        resetRotOffset.onTrue(runOnce({
            rotationOffset = Drivetrain.getPose().rotation
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

    fun setRumble(driver: Double) {
        driverController.setRumble(GenericHID.RumbleType.kBothRumble, driver)
    }

    fun rumbleBlip(strength: Supplier<Double>): Command {
        return runOnce({ setRumble(strength.get()) })
            .andThen(Commands.waitSeconds(strength.get()))
            .andThen(runOnce({ setRumble(strength.get()) }))
    }

    fun getGunnerSlider(): Double {
        return (-gunnerController.getRawAxis(3) + 1.0) / 2.0
    }

    fun getFieldOriented(): Boolean {
        return !robotOriented.asBoolean
    }

    fun maxSpeed(): Double {
        return if (outtake.asBoolean && !passing) {
            0.1
        } else {
            1.0
        }
    }
}