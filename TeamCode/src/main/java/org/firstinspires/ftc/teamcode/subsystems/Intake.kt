package org.firstinspires.ftc.teamcode.subsystems

import club.bitflip.utils.hardware.MotorEx
import com.pedropathing.ivy.CommandBuilder
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.commands.Commands.waitMs
import com.pedropathing.ivy.groups.Groups.sequential
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

class Intake(hardwareMap: HardwareMap, telemetry: Telemetry) : Subsystem(hardwareMap, telemetry) {

    enum class Mode(val power: Double) {
        OFF(0.0),
        SHOOTING(0.8),
        ON(0.9),
        REVERSED(-0.5)
    }

    private val motorL = MotorEx("intakeL").zeroed().float().reverse()
    private val motorR = MotorEx("intakeR").zeroed().float()

    var reversed = false
    var mode = Mode.OFF
        set(newMode) {
            field = newMode
            motorL.power = newMode.power
            motorR.power = newMode.power
        }

    init {
        motorL.setCurrentAlert(2.0)
    }

    override fun loop() {
        if (mode == Mode.ON) {
            if (reversed) mode = Mode.REVERSED
        }

        if (mode == Mode.REVERSED && !reversed) {
            mode = Mode.ON
        }

        telemetry.addData("Intake Mode", mode.name)
    }

    fun on() {
        mode = Mode.ON
    }

    fun off() {
        mode = Mode.OFF
    }

    fun toggle() {
        mode = if (mode == Mode.ON) {
            Mode.OFF
        } else {
            Mode.ON
        }
    }
}