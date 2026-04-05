using UnityEngine;

public class YachtVisualController : MonoBehaviour
{
    [Header("References")]
    public Transform rudder;
    public Transform propellerLeft;
    public Transform propellerRight;

    [Header("Animation Settings")]
    public float maxPropellerRPM = 300f;   // adjust for visual speed
    public float rudderMaxAngle = 35f;     // degrees

    // Values updated from PythonTCPReceiver
    [HideInInspector] public float throttle;      // 0..1 (or negative for reverse)
    [HideInInspector] public float rudderAngle;   // degrees

    void Update()
    {
        AnimatePropellers();
        AnimateRudder();
    }

    void AnimatePropellers()
    {
        if (propellerLeft == null || propellerRight == null)
            return;

        // Convert throttle to rotation speed
        float rpm = throttle * maxPropellerRPM;
        float degreesPerSecond = rpm * 6f; // 360° * RPM / 60

        // Rotate both propellers
        propellerLeft.Rotate(Vector3.forward, degreesPerSecond * Time.deltaTime, Space.Self);
        propellerRight.Rotate(Vector3.forward, degreesPerSecond * Time.deltaTime, Space.Self);
    }

    void AnimateRudder()
    {
        if (rudder == null)
            return;

        // Rudder rotates around Y axis
        float clampedAngle = Mathf.Clamp(rudderAngle, -rudderMaxAngle, rudderMaxAngle);
        rudder.localRotation = Quaternion.Euler(0f, clampedAngle, 0f);
    }
}