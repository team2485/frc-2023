package frc.util;

import edu.wpi.first.math.MathUtil;

/**
 * directly ripped from ncsariowan's BufferZone from the now deprecated WarlordsLib.
 * 
 */
public class BufferZone {

    private double m_minOutput, m_maxOutput;

    private double m_minPosition, m_maxPosition;

    private double m_bufferSize;

    /**
     * Buffers an output based on position limits
     * @param minOutput the minimum allowable output
     * @param maxOutput the maximum allowable output
     * @param minPosition the minimum allowable position
     * @param maxPosition the maximum allowable position
     * @param bufferSize the position range at which the buffer will begin linearly limiting the output
     */
    public BufferZone(double minOutput, double maxOutput, double minPosition, double maxPosition, double bufferSize) {
        this.m_minOutput = minOutput;
        this.m_maxOutput = maxOutput;
        this.m_minPosition = minPosition;
        this.m_maxPosition = maxPosition;
        this.m_bufferSize = bufferSize;
    }

    /**
     * Get the next buffered value
     * @param input the desired output
     * @param position the current position of the system
     * @return a clamped value based on the position of the system.
     */
    public double get(double input, double position) {
        return MathUtil.clamp(input,
                Math.min(Math.max(m_minOutput, (m_minOutput / m_bufferSize) * (position - m_minPosition)), 0),
                Math.max(Math.min(m_maxOutput, - (m_maxOutput /m_bufferSize) * (position - m_maxPosition)), 0));
    }
}