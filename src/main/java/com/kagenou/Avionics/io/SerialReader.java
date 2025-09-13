package com.kagenou.Avionics.io;

import com.fazecast.jSerialComm.SerialPort;
import com.fazecast.jSerialComm.SerialPortEvent;
import com.fazecast.jSerialComm.SerialPortMessageListener;

/**
 * Live IMU feed over a serial port. Reads newline-delimited lines, parses them
 * with {@link QuatParser}, and publishes to an {@link AttitudeState}.
 *
 * <p>Replaces the old {@code serialComm} class, fixing its parse bug (it assumed
 * a {@code "w:.. x:.."} format the firmware never sent). A robust reconnect loop
 * and port auto-detection are intentionally deferred to later milestones; this is
 * the minimal correct reader for M1.
 */
public final class SerialReader implements AttitudeSource {
    private final AttitudeState state;
    private final String portName;
    private final int baud;
    private SerialPort port;

    public SerialReader(AttitudeState state, String portName, int baud) {
        this.state = state;
        this.portName = portName;
        this.baud = baud;
    }

    /**
     * Picks the most likely IMU serial port: a descriptor matching the ST-Link VCP
     * or a common USB-serial bridge, else the sole port if exactly one is present,
     * else {@code null}. Kills the old hardcoded {@code COM9}.
     */
    public static String autoDetect() {
        SerialPort[] ports = SerialPort.getCommPorts();
        for (SerialPort p : ports) {
            String d = (p.getDescriptivePortName() + " " + p.getPortDescription()).toLowerCase();
            if (d.contains("stlink") || d.contains("st-link") || d.contains("usb serial")
                    || d.contains("usb-serial") || d.contains("vcp")
                    || d.contains("ch340") || d.contains("cp210") || d.contains("ftdi")) {
                return p.getSystemPortName();
            }
        }
        return ports.length == 1 ? ports[0].getSystemPortName() : null;
    }

    @Override
    public void start() {
        port = SerialPort.getCommPort(portName);
        port.setBaudRate(baud);
        if (!port.openPort()) {
            System.out.println("Java: failed to open " + portName + ". Available ports:");
            for (SerialPort p : SerialPort.getCommPorts()) {
                System.out.println("    " + p.getSystemPortName() + " - " + p.getDescriptivePortName());
            }
            return;
        }
        System.out.println("Java: connected to " + describe());

        port.addDataListener(new SerialPortMessageListener() {
            @Override
            public byte[] getMessageDelimiter() {
                return new byte[]{'\n'};
            }

            @Override
            public boolean delimiterIndicatesEndOfMessage() {
                return true;
            }

            @Override
            public int getListeningEvents() {
                return SerialPort.LISTENING_EVENT_DATA_RECEIVED;
            }

            @Override
            public void serialEvent(SerialPortEvent event) {
                String line = new String(event.getReceivedData()).trim();
                state.recordLine();
                float[] q = QuatParser.parse(line);
                if (q != null) {
                    state.update(q[0], q[1], q[2], q[3]);
                } else {
                    state.recordParseError();
                }
            }
        });
    }

    @Override
    public void stop() {
        if (port != null && port.isOpen()) {
            port.removeDataListener();
            port.closePort();
        }
    }

    @Override
    public String describe() {
        return "serial " + portName + " @" + baud;
    }
}
