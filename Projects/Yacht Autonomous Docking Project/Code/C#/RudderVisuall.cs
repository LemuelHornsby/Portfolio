using UnityEngine;

public class RudderVisual : MonoBehaviour
{
    public YachtControlSurface control;
    public Transform rudderMesh;

    void Update()
    {
        float angle = control.CurrentRudderAngleDeg;
        rudderMesh.localRotation = Quaternion.Euler(0f, angle, 0f);
    }
}