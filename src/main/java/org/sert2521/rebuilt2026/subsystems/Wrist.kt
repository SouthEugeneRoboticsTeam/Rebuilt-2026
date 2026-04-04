package org.sert2521.rebuilt2026.subsystems

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.rebuilt2026.ElectronicIDs
import org.sert2521.rebuilt2026.GrintakeConstants
import org.sert2521.rebuilt2026.TelemetryConstants
import org.sert2521.rebuilt2026.util.ZoneUtil
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper
import yams.telemetry.MechanismTelemetry

object Wrist : SubsystemBase() {
    private val wristMotorConfig = SmartMotorControllerConfig(this)
        .withClosedLoopController(GrintakeConstants.WRIST_P, 0.0, GrintakeConstants.WRIST_D)
        .withTelemetry("Wrist Motor", TelemetryConstants.GRINTAKE_TELEMETRY)
        .withMotorInverted(true)
        .withStatorCurrentLimit(Amps.of(40.0))
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withGearing(GrintakeConstants.wristGearing)

    private val wristMotor = SparkMax(ElectronicIDs.GRINTAKE_WRIST_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)

    private val wristSMC = SparkWrapper(
        wristMotor, DCMotor.getNEO(1),
        wristMotorConfig
    )

    private var setpoint = GrintakeConstants.stowPosition
    private var rezero = false

    private val currentDebouncer = Debouncer(0.2, Debouncer.DebounceType.kRising)

    private val telemetry = MechanismTelemetry()

    init {
        telemetry.setupTelemetry("Wrist", wristSMC)
    }

    override fun periodic() {
        wristSMC.updateTelemetry()
        if (!rezero) {
            wristSMC.setPosition(setpoint)
        }
    }

    fun up(): Command {
        return runOnce {
            rezero = false
            setpoint = GrintakeConstants.stowPosition
        }
    }

    fun down(): Command {
        return runOnce {
            setpoint = GrintakeConstants.intakePosition
        }
    }

    fun downSafeDepot(): Command {
        return (down()
            .andThen(Commands.idle())
            .until(ZoneUtil::isDepot)
            .andThen(toDepotInter())
            .andThen(Commands.idle())
            .until{ !ZoneUtil.isDepot() })
            .repeatedly()
    }

    fun toDepotInter(): Command {
        return runOnce {
            setpoint = GrintakeConstants.depotInter
        }
    }

    fun toDepot(): Command {
        return runOnce {
            setpoint = GrintakeConstants.depotPosition
        }
    }

    fun reZero(): Command {
        return runOnce {
            rezero = true
            wristSMC.dutyCycle = GrintakeConstants.REZERO_SPEED
            currentDebouncer.calculate(false)
        }.andThen(
            Commands.idle()
        ).until {
            currentDebouncer.calculate(
                wristSMC.statorCurrent > GrintakeConstants.reZeroThreshold
            )
        }.andThen(
            runOnce {
                wristSMC.setEncoderPosition(Rotations.zero())
                rezero = false
            }
        ).andThen(
            up()
        )
    }


}