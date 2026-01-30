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

    // TODO: Remove this as it is the same as the wristSMC that you wrote later
    private val smc = SparkWrapper(wristMotor, DCMotor.getNEO(1), wristMotorConfig)

    // TODO: This is a smart mechanism. Remove it if you can.
    //  Use the function I wrote for wristSMC.setPosition instead
    private val wristConfig = ArmConfig(smc)
        .withHardLimit(GrintakeConstants.hardMin, GrintakeConstants.hardMax)
        .withTelemetry("Wrist", SmartMotorControllerConfig.TelemetryVerbosity.LOW)
        .withStartingPosition(Rotations.of(0.0))
    private val wrist = Arm(wristConfig)


    // TODO: Fix capitalization on these two
    private val RollersSMC = SparkWrapper(
        rollerMotor, DCMotor.getNEO(1),
        rollerMotorConfig
    )

    private val WristSMC = SparkWrapper(
        wristMotor, DCMotor.getNEO(1),
        wristMotorConfig
    )

    private val telemetry = MechanismTelemetry()

    init{
        defaultCommand = stow()
        telemetry.setupTelemetry("Grintake", RollersSMC)
        telemetry.setupTelemetry("Grintake", WristSMC)
    }

    override fun periodic() {
        RollersSMC.updateTelemetry()
        WristSMC.updateTelemetry()
    }

    private fun setRollerMotor(dutyCycle: Double) {
        RollersSMC.dutyCycle = dutyCycle
    }

    private fun setWristMotorAngle(angle: Angle){
        WristSMC.setPosition(angle)
    }

    fun stow(): Command {
        // TODO: Add idle after and make it also stop the roller motor
        return runOnce{
            wrist.setAngle(GrintakeConstants.stowPosition)
        }
    }

    fun intake(): Command {
        // TODO: Add idle after
        return runOnce {
            wrist.setAngle(GrintakeConstants.intakePosition)
            setRollerMotor(GrintakeConstants.intakeSpeed)
        }
    }

    fun reverse(): Command {
        // TODO: Add idle after
        return runOnce {
            setRollerMotor(GrintakeConstants.reverseSpeed)
        }
    }
}