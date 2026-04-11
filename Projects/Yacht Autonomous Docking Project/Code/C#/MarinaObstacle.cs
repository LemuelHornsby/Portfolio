using UnityEngine;

public enum ObstacleKind
{
    Buoy,
    LifeBuoy,
    Ship,
    Wall,
    Dock
}

[DisallowMultipleComponent]
public class MarinaObstacle : MonoBehaviour
{
    [Header("Identity")]
    public int id = 0;
    public ObstacleKind kind = ObstacleKind.Buoy;

    [Header("RL / Collision Model")]
    [Tooltip("Circular safety radius in meters used by Python/RL")]
    public float safetyRadius = 3f;
}