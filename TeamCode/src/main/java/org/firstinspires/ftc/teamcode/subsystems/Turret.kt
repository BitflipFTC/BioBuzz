package org.firstinspires.ftc.teamcode.subsystems

import club.bitflip.utils.hardware.ServoEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

class Turret(hardwareMap: HardwareMap, telemetry: Telemetry) : Subsystem(hardwareMap, telemetry) {
    private val turret1 = ServoEx("turret1")
    private val turret2 = ServoEx("turret2")

    var position = 0.5
        set(newPosition) {
            field = newPosition
            turret1.position = newPosition
            turret2.position = newPosition
        }

    var degrees: Double
        get() {
            return (position - 0.5) * (2 * 160)
        }
        set(newDegrees) {
            position = ((newDegrees) / (2 * 160)) + 0.5
        }

    override fun loop () {
        telemetry.addData("Turret angle", degrees)
    }
}