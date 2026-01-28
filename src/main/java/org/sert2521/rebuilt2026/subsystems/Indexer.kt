package org.sert2521.rebuilt2026.subsystems

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.wpilibj.motorcontrol.Spark
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.rebuilt2026.ElectronicIDs
import org.sert2521.rebuilt2026.IndexerConstants
import yams.motorcontrollers.SmartMotorController
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper
import yams.telemetry.MechanismTelemetry

object Indexer : SubsystemBase() {
    private val indexerMotor = SparkMax(ElectronicIDs)

    private val IndexerMotorConfig = SmartMotorControllerConfig(this)
        .withGearing(IndexerConstants.IndexerGearing)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BREAK)
        .withTelemetry("Indexer Motor", SmartMotorControllerConfig.TelemetryVerbosity.LOW)
        .withStatorCurrentLimit(Amps.of(0.0))
        .withMotorInverted(false)
        .withControlMode(SmartMotorControllerConfig.ControlMode.OPEN_LOOP)

    private val indexerSMC = SparkWrapper(indexerMotor, DCMotor.Neo(0), IndexerMotorConfig)

    private val kickerMotor = SparkMax(ElectronicIDs.KICKER_MOTOR_ID, SparkLowLevel.MotorType)

    private val kickerMotorConfig = SmartMotorControllerConfig(this)
        .withGearing(IndexerConstants.kickerGearing)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withTelemetry("Kicker Motor", SmartMotorControllerConfig.TelemetryVerbosity.LOW)
        .withStatorCurrentLimit(Amps.of(0.0))
        .withMotorInverted(false)
        .withControlMode(SmartMotorControllerConfig.ControlMode.OPEN_LOOP)

    private val kickerSMC = SparkWrapper(kickerMotor, DCmotor.getNEO(0), kickerMotorConfig)

    private val telemetry = MechanismTelemetry()

    init
}