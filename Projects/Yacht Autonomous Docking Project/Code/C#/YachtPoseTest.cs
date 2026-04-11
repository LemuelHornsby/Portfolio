using UnityEngine;

public class YachtPoseTester : MonoBehaviour
{
    public YachtPoseApplier poseApplier;

    [Header("Test motion settings")]
    public float surgeSpeed = 2f;   // forward along +X
    public float swaySpeed = 0f;    // sideways along +Z
    public float yawRateDeg = 0f;   // rotation around +Y

    private float x = 0f;   // surge
    private float y = 0f;   // sway
    private float psi = 0f; // heading (radians)

    void Update()
    {
        float dt = Time.deltaTime;

        // Integrate surge and sway
        x += surgeSpeed * dt;   // forward
        y += swaySpeed * dt;    // sideways

        // Integrate heading
        psi += Mathf.Deg2Rad * yawRateDeg * dt;

        // Apply pose to the yacht
        poseApplier.ApplyPose(x, y, psi);
    }
}