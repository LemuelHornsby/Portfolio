using UnityEngine;

public class ActuatorLocator : MonoBehaviour
{
    public Transform shipPivot;
    public Transform rudder;
    public Transform bowThruster;

    void Start()
    {
        Vector3 r = shipPivot.InverseTransformPoint(rudder.position);
        Vector3 b = shipPivot.InverseTransformPoint(bowThruster.position);

        Debug.Log($"Rudder local (ShipPivot frame): x={r.x:F3}, z={r.z:F3}");
        Debug.Log($"BowThruster local (ShipPivot frame): x={b.x:F3}, z={b.z:F3}");

        Debug.Log($"ShipPivot axes: +X={shipPivot.right}, +Z={shipPivot.forward}");
    }
}
