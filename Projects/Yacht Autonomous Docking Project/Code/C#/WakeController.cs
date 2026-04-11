using UnityEngine;

public class WakeController : MonoBehaviour
{
    public YachtControlSurface control;
    public TrailRenderer wakeTrail;

    [Header("Wake intensity settings")]
    public float minWidth = 0f;
    public float maxWidth = 1.5f;

    void Update()
    {
        if (wakeTrail == null || control == null)
            return;

        // Wake width scales with throttle magnitude
        float t = Mathf.Abs(control.CurrentThrottle);
        wakeTrail.startWidth = Mathf.Lerp(minWidth, maxWidth, t);
        wakeTrail.endWidth = Mathf.Lerp(minWidth * 0.5f, maxWidth * 0.5f, t);
    }
}