package org.sert2521.rebuilt2026.subsystems

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.rebuilt2026.ElectronicIDs
import org.sert2521.rebuilt2026.HoodedShooterConstants
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper
import yams.telemetry.MechanismTelemetry

object HoodedShooter : SubsystemBase() {
        private val motorLeader = SparkMax(ElectronicIDs.SHOOTER_LEADER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)
        private val motorFollower = SparkMax(ElectronicIDs.SHOOTER_FOLLOWER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)
        private val motorRoller = SparkMax(ElectronicIDs.SHOOTER_ROLLER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)

        private val motorConfigLeader = SmartMotorControllerConfig(this)
            .withClosedLoopController(HoodedShooterConstants.P, 0.0, HoodedShooterConstants.D)
            .withFeedforward(SimpleMotorFeedforward(HoodedShooterConstants.S, HoodedShooterConstants.V,
                HoodedShooterConstants.A))
            .withGearing(HoodedShooterConstants.shooterGearing)
            .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
            .withTelemetry("Shooter Motor Leader", SmartMotorControllerConfig.TelemetryVerbosity.LOW)
            .withStatorCurrentLimit(Amps.of(40.0))
            .withMotorInverted(false)

        private val motorConfigFollower = motorConfigLeader.clone()
            .withMotorInverted(true)
            .withTelemetry("Shooter Motor Follower", SmartMotorControllerConfig.TelemetryVerbosity.LOW)

        private val motorConfigRoller = SmartMotorControllerConfig(this)
            .withControlMode(SmartMotorControllerConfig.ControlMode.OPEN_LOOP)
            .withMotorInverted(false)
            .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(40.0))
            .withGearing(HoodedShooterConstants.rollerGearing)
            .withTelemetry("Shooter Motor Roller", SmartMotorControllerConfig.TelemetryVerbosity.LOW)

        private val leaderSMC = SparkWrapper(motorLeader, DCMotor.getNEO(1), motorConfigLeader)
        private val followerSMC = SparkWrapper(motorFollower, DCMotor.getNEO(1), motorConfigFollower)
        private val rollerSMC = SparkWrapper(motorRoller, DCMotor.getNEO(1), motorConfigRoller)
        private val telemetry = MechanismTelemetry()

        private var shooterSetpoint = RPM.zero()

        init {
            // defaultCommand = holdCommand(::shooterSetpoint)

            telemetry.setupTelemetry("HoodedShooter", leaderSMC)
            telemetry.setupTelemetry("HoodedShooter", followerSMC)
            telemetry.setupTelemetry("HoodedShooter", rollerSMC)
        }

        override fun periodic() {
            leaderSMC.updateTelemetry()
            followerSMC.updateTelemetry()
            rollerSMC.updateTelemetry()
        }

    override fun simulationPeriodic() {
        leaderSMC.simIterate()
        followerSMC.simIterate()
        rollerSMC.simIterate()
    }

    private fun setVelocitiesCommand(flywheelsVelocity: AngularVelocity, rollerDutyCycle:Double): Command {
        return runOnce {
            leaderSMC.setVelocity(flywheelsVelocity)
            rollerSMC.dutyCycle = rollerDutyCycle
        }
    }
}