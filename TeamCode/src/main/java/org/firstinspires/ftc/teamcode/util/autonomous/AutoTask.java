package org.firstinspires.ftc.teamcode.util.autonomous;

import org.firstinspires.ftc.teamcode.hardware.DukHardwareMap;

public interface AutoTask {
    abstract class Base {
        public static DukHardwareMap _hardwareMap;
    }

    void initialize();
    void execute();
    boolean shouldTerminate();
    void onTerminate();
    boolean runSynchronous();
}