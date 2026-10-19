package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

abstract class Subsystem (val hardwareMap: HardwareMap, val telemetry: Telemetry) {
    open fun loop() {}
}