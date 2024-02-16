package org.firstinspires.ftc.teamcode.hardware.wrappers;

import org.firstinspires.ftc.teamcode.util.DashboardInterface;
import org.firstinspires.ftc.teamcode.util.DukConstants;

import java.util.HashMap;
import java.util.Objects;

public class C_TelemetryLoggingBuffer {
    String name;
    HashMap<String, Object> data = new HashMap<>();
    HashMap<String, Object> oldData = new HashMap<>();

    public C_TelemetryLoggingBuffer(String _name) {
        name = _name;
    }

    public void push(String label, Object value) {
        if (data.containsKey(label))
            System.out.println("Telemetry packet \"" + name + "\" already contains " + label + "!");
        else if (Objects.equals(oldData.get(label), value) && DukConstants.DEBUG.OPTIMIZE_PACKETS) return;
        data.put(label, value);
    }

    //Maybe putAll into the DashboardInterface.bufferPacket instead of sending immediately?
    public void dispatch() {
        if (data.isEmpty()) return;
        data.forEach((_label, _object) -> DashboardInterface.bufferPacket.put(name + ":" + _label, _object));
        oldData.putAll(data);
        data.clear();
    }
}
