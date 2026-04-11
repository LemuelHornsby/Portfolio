using UnityEngine;

public class YachtControlSurface : MonoBehaviour
{
    [Header("Normalized control inputs from Python")]
    [Range(-1f, 1f)] public float throttleCmd = 0f;   // -1 = full reverse, +1 = full ahead
    [Range(-1f, 1f)] public float rudderCmd = 0f;     // -1 = port, +1 = starboard

    [Header("Physical limits")]
    public float maxRudderAngle = 30f;   // degrees

    // Public getters for other scripts (propeller, rudder mesh, wake, etc.)
    public float CurrentThrottle => throttleCmd;
    public float CurrentRudderAngleDeg => rudderCmd * maxRudderAngle;

    /// <summary>
    /// Called by Python receiver to update control inputs.
    /// </summary>
    public void SetControl(float throttle, float rudder)
    {
        throttleCmd = Mathf.Clamp(throttle, -1f, 1f);
        rudderCmd = Mathf.Clamp(rudder, -1f, 1f);
    }
}