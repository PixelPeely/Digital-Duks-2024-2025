package org.firstinspires.ftc.teamcode.util.autonomous;

import org.firstinspires.ftc.teamcode.hardware.DukHardwareMap;

public interface AutonTask {
    abstract class Base {
        public static DukHardwareMap hMap;
    }

    void initialize();
    void execute();
    boolean shouldTerminate();
    void onTerminate();
    boolean runSynchronous();
}