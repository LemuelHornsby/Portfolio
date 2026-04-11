// 12/29/2025 AI-Tag
// This was created with the help of Assistant, a Unity Artificial Intelligence product.

using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Rigidbody))]
public class YachtController : MonoBehaviour
{
    public float maxThrust = 50f; // Maximum thrust force
    public float accelerationRate = 10f; // Rate of acceleration
    public float decelerationRate = 20f; // Rate of deceleration
    public float reverseSpeed = 20f; // Maximum reverse speed
    public float turnSpeed = 30f; // Turning speed

    private Rigidbody yachtRigidbody;
    private float currentThrust = 0f;
    private float steerInput = 0f;
    private bool isBraking = false;
    private bool isReversing = false;

    private void Awake()
    {
        yachtRigidbody = GetComponent<Rigidbody>();
        currentThrust = 0f; // Ensure thrust starts at 0
    }

    public void OnThrust(InputAction.CallbackContext context)
    {
        float thrustInput = context.ReadValue<float>();
        if (!isReversing)
        {
            currentThrust = Mathf.MoveTowards(currentThrust, thrustInput * maxThrust, accelerationRate * Time.deltaTime);
        }
    }

    public void OnBrake(InputAction.CallbackContext context)
    {
        isBraking = context.ReadValueAsButton();
        if (isBraking)
        {
            currentThrust = Mathf.MoveTowards(currentThrust, 0, decelerationRate * Time.deltaTime);
        }
    }

    public void OnReverse(InputAction.CallbackContext context)
    {
        isReversing = context.ReadValueAsButton();
        if (isReversing)
        {
            currentThrust = Mathf.MoveTowards(currentThrust, -reverseSpeed, accelerationRate * Time.deltaTime);
        }
    }

    public void OnSteer(InputAction.CallbackContext context)
    {
        Vector2 steerVector = context.ReadValue<Vector2>();
        steerInput = steerVector.x; // Extract the X-axis value for yaw control
    }

    private void FixedUpdate()
    {
        // Apply thrust along the X-axis
        Vector3 forwardMovement = transform.right * currentThrust * Time.fixedDeltaTime;
        yachtRigidbody.MovePosition(yachtRigidbody.position + forwardMovement);

        // Apply steering (yaw rotation)
        float rotation = steerInput * turnSpeed * Time.fixedDeltaTime;
        yachtRigidbody.MoveRotation(yachtRigidbody.rotation * Quaternion.Euler(0, rotation, 0));
    }
}