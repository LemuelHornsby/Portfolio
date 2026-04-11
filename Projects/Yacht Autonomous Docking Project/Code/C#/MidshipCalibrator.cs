using UnityEngine;

public class MidshipCalibrator : MonoBehaviour
{
    public Transform shipPivot;   // Driven by Python
    public Transform yachtMesh;   // "myyacht" (child of ShipPivot)
    public Transform bowMarker;   // child of myyacht, placed at bow tip
    public Transform sternMarker; // child of myyacht, placed at stern tip

    [ContextMenu("Center Mesh On ShipPivot Midship")]
    public void CenterMeshOnShipPivotMidship()
    {
        if (!shipPivot || !yachtMesh || !bowMarker || !sternMarker)
        {
            Debug.LogError("[MidshipCalibrator] Assign shipPivot, yachtMesh, bowMarker, sternMarker.");
            return;
        }

        // Midpoint in WORLD space between the two markers
        Vector3 midWorld = 0.5f * (bowMarker.position + sternMarker.position);

        // Midpoint expressed in shipPivot LOCAL coordinates
        Vector3 midLocalInPivot = shipPivot.InverseTransformPoint(midWorld);

        // Shift the mesh so that the midpoint becomes (0,0,0) in shipPivot local space
        yachtMesh.localPosition -= new Vector3(midLocalInPivot.x, 0f, midLocalInPivot.z);


        Debug.Log($"[MidshipCalibrator] midLocalInPivot={midLocalInPivot} -> yachtMesh.localPosition={yachtMesh.localPosition}");
    }
}
