package org.sert2521.rebuilt2026.subsystems

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.rebuilt2026.ElectronicIDs
import org.sert2521.rebuilt2026.GrintakeConstants
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper
import yams.telemetry.MechanismTelemetry

object Grintake : SubsystemBase() {
    private val rollerMotor = SparkMax(ElectronicIDs.GRINTAKE_ROLLER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)
    private val wristMotor = SparkMax(ElectronicIDs.GRINTAKE_WRIST_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)

    private val rollerMotorConfig = SmartMotorControllerConfig(this)
        .withGearing(GrintakeConstants.rollerGearing)
        .withMotorInverted(false)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withTelemetry("Roller Motor", SmartMotorControllerConfig.TelemetryVerbosity.LOW)
        .withControlMode(SmartMotorControllerConfig.ControlMode.OPEN_LOOP)
        .withStatorCurrentLimit(Amps.of(40.0))
    private val wristMotorConfig = SmartMotorControllerConfig(this)
        .withClosedLoopController(GrintakeConstants.WRIST_P, 0.0, GrintakeConstants.WRIST_D)
        .withTelemetry("Wrist Motor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withMotorInverted(true)
        .withStatorCurrentLimit(Amps.of(40.0))
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withGearing(GrintakeConstants.wristGearing)

    private val rollerSMC = SparkWrapper(
        rollerMotor, DCMotor.getNEO(1),
        rollerMotorConfig
    )
    private val wristSMC = SparkWrapper(
        wristMotor, DCMotor.getNEO(1),
        wristMotorConfig
    )

    private val currentDebouncer = Debouncer(0.5, Debouncer.DebounceType.kRising)

    private val telemetry = MechanismTelemetry()

    init {
        defaultCommand = stow()

        telemetry.setupTelemetry("Grintake", rollerSMC)
        telemetry.setupTelemetry("Grintake", wristSMC)
    }

    override fun periodic() {
        rollerSMC.updateTelemetry()
        wristSMC.updateTelemetry()
    }

    private fun setRollerMotor(dutyCycle: Double) {
        rollerSMC.dutyCycle = dutyCycle
    }

    private fun setWristMotorAngle(angle: Angle) {
        wristSMC.setPosition(angle)
    }

    fun stow(): Command {
        return runOnce {
            setWristMotorAngle(GrintakeConstants.stowPosition)
            setRollerMotor(0.0)
        }.andThen(
            Commands.idle()
        )
    }

    fun reZero():Command{
        return runOnce {
            setRollerMotor(0.0)
            wristSMC.dutyCycle = GrintakeConstants.REZERO_SPEED
            currentDebouncer.calculate(false)
        }.andThen(
            Commands.idle()
        ).until{
            currentDebouncer.calculate(
                wristSMC.statorCurrent > GrintakeConstants.reZeroThreshold
            )
        }.andThen(
            runOnce{
                wristSMC.setEncoderPosition(Rotations.zero())
            }
        ).andThen(
            stow()
        )
    }

    fun intake(): Command {
        return runOnce {
            setWristMotorAngle(GrintakeConstants.intakePosition)
            setRollerMotor(GrintakeConstants.INTAKE_SPEED)
        }.andThen(
            Commands.idle()
        )
    }

    fun reverse(): Command {
        return runOnce {
            setRollerMotor(GrintakeConstants.REVERSE_SPEED)
        }.andThen(
            Commands.idle()
        )
    }
}