using UnityEngine;

public class DockGoalGizmo : MonoBehaviour
{
    public Color gizmoColor = Color.green;
    public float radius = 1.5f;

    void OnDrawGizmos()
    {
        Gizmos.color = gizmoColor;
        Gizmos.DrawSphere(transform.position, radius);

        // Draw forward direction
        Gizmos.color = Color.blue;
        Gizmos.DrawLine(transform.position, transform.position + transform.forward * 5f);
    }
}