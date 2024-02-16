package org.firstinspires.ftc.teamcode.hardware;

public interface CachedPeripheral {
    /**
     * Send cache to true hardware
     */
    void dispatchCache();

    /**
     * Retrieve and cache information from hardware
     */
    void refreshCache();

    /**
     * Enable/disable access to hardware
     */
    void allowDispatch(boolean state);

    /**
     * Is the true hardware non-null?
     */
    boolean isValid();
}