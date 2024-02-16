package org.firstinspires.ftc.teamcode.util.autonomous;

public interface AutonTask {
    void initialize();
    void execute();
    boolean shouldTerminate();
    void onTerminate();
    boolean runAsynchronous();
}
