package org.sert2521.rebuilt2026.subsystems

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.hal.simulation.AnalogGyroDataJNI.setAngle
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.KilogramSquareMeters
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.DutyCycle
import edu.wpi.first.wpilibj.motorcontrol.Spark
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.rebuilt2026.ElectronicIDs
import org.sert2521.rebuilt2026.GrintakeConstants
import yams.gearing.MechanismGearing
import yams.mechanisms.config.ArmConfig
import yams.mechanisms.positional.Arm
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
        .withTelemetry("Grintake Roller Motor", SmartMotorControllerConfig.TelemetryVerbosity.LOW)
        .withStatorCurrentLimit(Amps.of(40.0))

    private val wristMotorConfig = SmartMotorControllerConfig(this)
        .withTelemetry("Grintake Wrist Motor", SmartMotorControllerConfig.TelemetryVerbosity.LOW)
        .withMotorInverted(false)
        .withStatorCurrentLimit(Amps.of(40.0))
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withGearing(GrintakeConstants.wristGearing)

    private val rollersSMC = SparkWrapper(
        rollerMotor, DCMotor.getNEO(1),
        rollerMotorConfig
    )

    private val wristSMC = SparkWrapper(
        wristMotor, DCMotor.getNEO(1),
        wristMotorConfig
    )

    private val telemetry = MechanismTelemetry()

    init{
        defaultCommand = stow()
        telemetry.setupTelemetry("Grintake", rollersSMC)
        telemetry.setupTelemetry("Grintake", wristSMC)
    }

    override fun periodic() {
        rollersSMC.updateTelemetry()
        wristSMC.updateTelemetry()
    }

    private fun setRollerMotor(dutyCycle: Double) {
        rollersSMC.dutyCycle = dutyCycle
    }

    private fun setWristMotorAngle(angle: Angle){
        wristSMC.setPosition(angle)
    }

    fun stow(): Command {
        return runOnce{
            setWristMotorAngle(GrintakeConstants.stowPosition)
            setRollerMotor(0.0)
        }.andThen(
            Commands.idle()
        )
    }

    fun intake(): Command {
        return runOnce {
            setWristMotorAngle(GrintakeConstants.intakePosition)
            setRollerMotor(GrintakeConstants.intakeSpeed)
        }.andThen(
            Commands.idle()
        )
    }

    fun reverse(): Command {
        return runOnce {
            setRollerMotor(GrintakeConstants.reverseSpeed)
        }.andThen(
            Commands.idle()
        )
    }
}