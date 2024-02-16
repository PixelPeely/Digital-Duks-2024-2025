package org.firstinspires.ftc.teamcode.hardware;

public interface CachedSubsystem {
    //Send caches to all hardware
    void dispatchAllCaches();
    //Retrieve and cache information from all hardware
    void refreshAllCaches();
    //Debug
    void pushTelemetry();
    //Call allowDispatch on all peripherals
    void allowDispatch(boolean state);
}
