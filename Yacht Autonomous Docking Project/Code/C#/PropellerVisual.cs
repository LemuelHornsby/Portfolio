using UnityEngine;

public class PropellerVisual : MonoBehaviour
{
    public Transform pivotL;   // Parent of Propeller_L
    public Transform pivotR;   // Parent of Propeller_R

    private float rpmL = 0f;
    private float rpmR = 0f;

    // Called by MMGStateReceiver when new RPM arrives from Python
    public void SetRPM(float left, float right)
    {
        rpmL = left;
        rpmR = right;
    }

    void Update()
    {
        // Convert RPM to degrees per second
        float angleL = rpmL * Time.deltaTime;
        float angleR = rpmR * Time.deltaTime;

        // Rotate left propeller
        pivotL.Rotate(Vector3.right, angleL, Space.Self);

        // Rotate right propeller
        pivotR.Rotate(Vector3.right, angleR, Space.Self);
    }
}