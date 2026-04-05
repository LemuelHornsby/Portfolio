using UnityEngine;

public class WaveAudio : MonoBehaviour
{
    public Rigidbody rb;
    public AudioSource waveAudio;

    [Header("Tuning")]
    public float minVolume = 0f;
    public float maxVolume = 0.6f;
    public float speedInfluence = 0.05f;
    public float verticalInfluence = 0.3f;

    void Update()
    {
        // Forward speed
        float speed = rb.linearVelocity.magnitude;

        // Vertical motion (bobbing)
        float vertical = Mathf.Abs(rb.linearVelocity.y);

        // Combine influences
        float intensity = speed * speedInfluence + vertical * verticalInfluence;

        // Clamp and apply
        float volume = Mathf.Clamp(intensity, minVolume, maxVolume);
        waveAudio.volume = volume;

        // Optional: pitch variation
        waveAudio.pitch = 1f + (speed * 0.01f);
    }
}