using UnityEngine;

public class YachtPoseApplier : MonoBehaviour
{
    [Header("Assign the ROOT object (pivot at ship center)")]
    public Transform ShipPivot;

    [Header("Height of the yacht above water (Unity Y axis)")]
    public float waterlineHeight = 0f;

    public void ApplyPose(float x, float y, float psiRad)
{
    if (ShipPivot == null)
    {
        Debug.LogWarning("YachtPoseApplier: ShipPivot is not assigned.");
        return;
    }

    // Map Python surge/sway → Unity X/Z
    float unityX = x;
    float unityZ = y;

    // Convert heading directly
    float headingDeg = psiRad * Mathf.Rad2Deg;

    // Apply position
    ShipPivot.position = new Vector3(unityX, waterlineHeight, unityZ);

    // Apply rotation around Unity Y-axis
    ShipPivot.rotation = Quaternion.Euler(0f, headingDeg, 0f);
}
    
}



