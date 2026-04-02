package org.sert2521.rebuilt2026

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.sert2521.rebuilt2026.commands.HoodedShooterCommands
import org.sert2521.rebuilt2026.commands.JoystickDrive
import org.sert2521.rebuilt2026.commands.VisionRotationDrive
import org.sert2521.rebuilt2026.subsystems.Intake
import org.sert2521.rebuilt2026.subsystems.hooded_shooter.Flywheel
import org.sert2521.rebuilt2026.subsystems.Indexer
import org.sert2521.rebuilt2026.subsystems.Wrist
import org.sert2521.rebuilt2026.subsystems.drivetrain.Drivetrain
import org.sert2521.rebuilt2026.subsystems.hooded_shooter.Hood
import java.util.function.Supplier
import kotlin.jvm.optionals.getOrElse

object Input {
    private val driverController = CommandXboxController(0)
    private val gunnerController = CommandJoystick(1)

    private val intake = driverController.rightTrigger()
    private val wristDown = driverController.leftTrigger()
    private val reverseIntake = driverController.leftBumper()
    private val robotOriented = driverController.rightBumper()
    private val outtake = gunnerController.button(2)
    private val stopRev = gunnerController.button(8)

    private val reverseIndex = gunnerController.button(3) // Driver has control for reverse
    private val rev = gunnerController.button(4)
    private val revPass = gunnerController.button(9)

    private val manualIndex = gunnerController.button(1)

    private val visionAlign = driverController.x()

    private val resetRotOffset = driverController.y()
    private val resetRotReal = driverController.b()

    private val hoodDown = gunnerController.button(7)
    private val hoodUp = gunnerController.button(6)

    private val increaseFlywheel = gunnerController.button(11)
    private val decreaseFlywheel = gunnerController.button(16)

    private val startFlywheelLiveTuning = driverController.back()
    private val startFlywheelInterpolation = driverController.start()


    private var rotationOffset = Rotation2d.kZero


    init {

        outtake.whileTrue(Indexer.pulse().alongWith(runOnce({ Flywheel.startTimer() })).unless { !Flywheel.isRevved() })
        outtake.onFalse(runOnce({ Flywheel.stopTimer() }))
        stopRev.onTrue(HoodedShooterCommands.stop())

        hoodDown.onTrue(Hood.setPosition { ShooterConstants.hoodMin })
        hoodUp.onTrue(Hood.setPosition { ShooterConstants.hoodMax })

        intake.whileTrue(Intake.intake().alongWith(Indexer.manualIndex().asProxy()))
        wristDown.whileTrue(Wrist.down())
        wristDown.onFalse(Wrist.up())


        reverseIntake.whileTrue(Intake.reverse().alongWith(Indexer.reverse().asProxy()))
        rev.onTrue(HoodedShooterCommands.revAndTrackHub())
        revPass.whileTrue(HoodedShooterCommands.revAndTrackPass())

        reverseIndex.whileTrue(Indexer.reverse())

        manualIndex.whileTrue(Indexer.manualIndex())

        visionAlign.whileTrue(VisionRotationDrive(::getFieldOriented,
            { Drivetrain.rotationTo(OtherConstsants.currentHub).rotateBy(Rotation2d.k180deg) }
        ))

        increaseFlywheel.onTrue(runOnce({ OtherConstsants.flywheelLiveSetpoint += RPM.of(10.0) }))
        decreaseFlywheel.onTrue(runOnce({ OtherConstsants.flywheelLiveSetpoint -= RPM.of(10.0) }))

        startFlywheelLiveTuning.onTrue(HoodedShooterCommands.liveTuning())
        startFlywheelInterpolation.onTrue(HoodedShooterCommands.revAndTrackHub())

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

    fun setRumble(amount: Double) {
        driverController.setRumble(GenericHID.RumbleType.kBothRumble, amount)
    }

    fun rumbleBlip(strength: Supplier<Double>): Command {
        return runOnce({ setRumble(strength.get()) })
            .andThen(Commands.waitSeconds(strength.get()))
            .andThen(runOnce({ setRumble(strength.get()) }))
    }

    fun getGunnerSlider(): Double{
        return (-gunnerController.getRawAxis(3) + 1.0)/2.0
    }

    fun getFieldOriented():Boolean {
        return !robotOriented.asBoolean
    }

    fun maxSpeed():Double {
        return if (outtake.asBoolean){
            0.1
        } else {
            1.0
        }
    }

    fun getIntaking():Boolean {
        return intake.asBoolean
    }
}