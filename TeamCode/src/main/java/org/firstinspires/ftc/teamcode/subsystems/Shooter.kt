package org.firstinspires.ftc.teamcode.subsystems

import club.bitflip.utils.hardware.MotorEx
import club.bitflip.utils.hardware.ServoEx
import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

@Configurable
class Shooter(hardwareMap: HardwareMap, telemetry: Telemetry) : Subsystem(hardwareMap, telemetry) {

    companion object {
        @JvmField
        var gain = 0.00002
    }

    private val motor = MotorEx("shooter").reverse().float()
    private val hood = ServoEx("hood")
    private val hoodMinPos = 0.5
    private var error = 0.0
    private var lastError = 0.0

    private var output = 0.0
    private var tbh = 0.0

    var hoodPosition
        get() = hood.position
        set(newPos) {
            hood.position = newPos.coerceIn(hoodMinPos, 1.0)
        }

    var targetFlywheelRPM = 0.0
    var flywheelRPM = 0.0
        private set

    init {
        hood.position = hoodMinPos
    }

    override fun loop() {
        flywheelRPM = (motor.velocity / 28) * 60

        lastError = error
        error = targetFlywheelRPM - flywheelRPM

        if (error >= 60.0) {
            motor.power = 1.0
            output = targetFlywheelRPM * (1.0 / 5000.0)
            tbh = output
        } else {
            output += gain * error
            output = output.coerceIn(-0.05, 1.0)

            if ((error > 0 && lastError < 0) || (error < 0 && lastError > 0)) {
                output = 0.5 * (output + tbh)
                tbh = output
            }

            output = output.coerceIn(0.00, 1.0)

            motor.power = output
        }

        telemetry.run {
            addData("Target flywheel RPM", targetFlywheelRPM)
            addData("Actual flywheel RPM", flywheelRPM)
            addData("Hood position", hoodPosition)
            addData("Motor power", output)
        }
    }

    fun readyToShoot() = (error <= 100.0)
}