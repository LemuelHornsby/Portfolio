using UnityEngine;

public enum ForwardAxis { PlusX, MinusX, PlusZ, MinusZ }

[DisallowMultipleComponent]
public class StraightMoverWithAcceleration : MonoBehaviour
{
    [Header("Speed Settings")]
    [Tooltip("Maximum forward speed (m/s)")]
    public float maxSpeed = 3.0f;

    [Tooltip("Acceleration rate (m/s^2)")]
    public float acceleration = 1.0f;

    [Tooltip("Deceleration rate when slowing (m/s^2)")]
    public float deceleration = 2.0f;

    [Header("Bow Direction")]
    public ForwardAxis forwardAxis = ForwardAxis.PlusZ;

    [Header("Advanced")]
    [Tooltip("Apply small linear drag (0 = none)")]
    public float drag = 0.0f;

    private float currentSpeed = 0f;

    private void OnValidate()
    {
        maxSpeed = Mathf.Max(0f, maxSpeed);
        acceleration = Mathf.Max(0f, acceleration);
        deceleration = Mathf.Max(0f, deceleration);
        drag = Mathf.Max(0f, drag);
    }

    private Vector3 LocalAxis()
    {
        switch (forwardAxis)
        {
            case ForwardAxis.PlusX:  return Vector3.right;
            case ForwardAxis.MinusX: return Vector3.left;
            case ForwardAxis.PlusZ:  return Vector3.forward;
            case ForwardAxis.MinusZ: return Vector3.back;
            default: return Vector3.forward;
        }
    }

    private void Update()
    {
        float dt = Time.deltaTime;

        // Accelerate toward maxSpeed
        if (currentSpeed < maxSpeed)
        {
            currentSpeed += acceleration * dt;
            currentSpeed = Mathf.Min(currentSpeed, maxSpeed);
        }

        // Apply optional drag
        if (drag > 0f)
        {
            currentSpeed -= drag * currentSpeed * dt;
            currentSpeed = Mathf.Max(0f, currentSpeed);
        }

        // Determine forward direction
        Vector3 worldDir = transform.TransformDirection(LocalAxis());
        worldDir.y = 0f;

        if (worldDir.sqrMagnitude < 1e-8f)
            return;

        worldDir.Normalize();

        // Move
        transform.position += worldDir * currentSpeed * dt;
    }

    // Optional: Call this if you want to stop ship smoothly
    public void BrakeToStop()
    {
        currentSpeed -= deceleration * Time.deltaTime;
        currentSpeed = Mathf.Max(0f, currentSpeed);
    }
}