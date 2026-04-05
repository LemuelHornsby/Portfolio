// 12/29/2025 AI-Tag
// This was created with the help of Assistant, a Unity Artificial Intelligence product.

using System;
using UnityEditor;
using UnityEngine;

public class WaveSimulation : MonoBehaviour
{
    public float waveHeight = 2f; // Height of the waves
    public float waveFrequency = 0.5f; // Frequency of the waves
    public float waveSpeed = 2f; // Speed of the waves
    public float waveLength = 10f; // Length of the waves

    public float GetWaveHeight(Vector3 position, float time)
    {
        return waveHeight * Mathf.Sin((position.x + time * waveSpeed) / waveLength);
    }

    public Vector3 GetWaveNormal(Vector3 position, float time)
    {
        float dx = waveHeight * Mathf.Cos((position.x + time * waveSpeed) / waveLength) / waveLength;
        return new Vector3(-dx, 1, 0).normalized;
    }
}
