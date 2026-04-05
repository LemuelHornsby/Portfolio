using UnityEngine;

public class EngineSoundController : MonoBehaviour
{
    public AudioSource engineAudio;

    [Header("Sound Tuning")]
    public float minPitch = 0.8f;
    public float maxPitch = 2.2f;
    public float minVolume = 0.1f;
    public float maxVolume = 0.8f;

    // Updated from PythonTCPReceiver
    [HideInInspector] public float rpm;

    void Update()
    {
        // Normalize RPM (0 to 1)
        float t = Mathf.Clamp01(Mathf.Abs(rpm) / 1500f); // adjust denominator to match your maxRPM

        // Pitch scales with RPM
        engineAudio.pitch = Mathf.Lerp(minPitch, maxPitch, t);

        // Volume scales with throttle
        engineAudio.volume = Mathf.Lerp(minVolume, maxVolume, t);
    }
}