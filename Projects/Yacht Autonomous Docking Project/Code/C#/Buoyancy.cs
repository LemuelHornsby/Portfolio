using UnityEngine;
using Crest;

public class Buoyancy : MonoBehaviour
{
    public Transform[] buoyancyPoints;
    public float buoyancyStrength = 10f;

    Rigidbody rb;
    SampleHeightHelper _sampleHeightHelper;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        _sampleHeightHelper = new SampleHeightHelper();
    }

    void FixedUpdate()
    {
        foreach (Transform point in buoyancyPoints)
        {
            Vector3 pos = point.position;

            // --- CREST WAVE HEIGHT QUERY ---
            float waterHeight = 0f;
            if (OceanRenderer.Instance != null)
            {
                _sampleHeightHelper.Init(pos, 0f, true);
                _sampleHeightHelper.Sample(out waterHeight);
            }

            // Apply buoyancy if point is below water
            if (pos.y < waterHeight)
            {
                float displacement = waterHeight - pos.y;

                Vector3 force = Vector3.up * displacement * buoyancyStrength;

                rb.AddForceAtPosition(force, pos, ForceMode.Force);
            }
        }
    }
}