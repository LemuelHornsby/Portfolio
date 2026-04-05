// 1/1/2026 AI-Tag
// This was created with the help of Assistant, a Unity Artificial Intelligence product.

using UnityEngine;

public class PropellerEffectsController : MonoBehaviour
{
    public ParticleSystem propellerWash; // Particle system for underwater propeller wash
    public TrailRenderer wakeTrail; // Trail renderer for water wake
    public Rigidbody yachtRigidbody; // Rigidbody of the yacht
    public Transform waterSurface; // Reference to the water surface transform
    public float speedThreshold = 5f;

    void Update()
    {
        float speed = yachtRigidbody.linearVelocity.magnitude;

        // Adjust particle system emission rate based on speed
        var emission = propellerWash.emission;
        emission.rateOverTime = speed > speedThreshold ? speed * 10 : 0;

        // Adjust trail renderer width based on speed
        wakeTrail.widthMultiplier = Mathf.Clamp(speed / 10f, 0.1f, 1f);

        // Position the wake trail at the water surface
        Vector3 wakePosition = new Vector3(transform.position.x, waterSurface.position.y, transform.position.z);
        wakeTrail.transform.position = wakePosition;
    }
}