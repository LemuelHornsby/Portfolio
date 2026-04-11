using UnityEngine;
using UnityEngine.InputSystem;

public class ShipController : MonoBehaviour
{
    public float maxForwardSpeed = 20f;
    public float acceleration = 10f;
    public float turnSpeed = 30f;
    public float brakeStrength = 5f;

    private Rigidbody rb;

    // Input values
    private Vector2 moveInput;
    private float throttleInput;
    private float brakeInput;
    private float yawInput;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    // Called by Player Input
    public void OnMove(InputAction.CallbackContext ctx)
    {
        moveInput = ctx.ReadValue<Vector2>();
    }

    public void OnThrottle(InputAction.CallbackContext ctx)
    {
        throttleInput = ctx.ReadValue<float>();
    }

    public void OnBrake(InputAction.CallbackContext ctx)
    {
        brakeInput = ctx.ReadValue<float>();
    }

    public void OnYaw(InputAction.CallbackContext ctx)
    {
        yawInput = ctx.ReadValue<float>();
    }

    void FixedUpdate()
    {
        // --- Forward movement ---
        Vector3 forward = transform.forward * moveInput.y;
        float targetSpeed = throttleInput * maxForwardSpeed;

        Vector3 desiredVelocity = forward * targetSpeed;
        Vector3 velocityChange = desiredVelocity - rb.linearVelocity;
        rb.AddForce(velocityChange * acceleration, ForceMode.Acceleration);

        // --- Turning (Yaw) ---
        float turn = yawInput * turnSpeed;
        rb.AddTorque(Vector3.up * turn, ForceMode.Acceleration);

        // --- Braking ---
        if (brakeInput > 0.1f)
        {
            rb.linearVelocity *= (1f - brakeStrength * Time.fixedDeltaTime);
            rb.angularVelocity *= (1f - brakeStrength * Time.fixedDeltaTime);
        }
    }
}