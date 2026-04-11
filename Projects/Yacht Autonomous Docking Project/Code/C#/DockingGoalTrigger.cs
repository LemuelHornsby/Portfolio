using UnityEngine;

public class DockGoalTrigger : MonoBehaviour
{
    public static bool ShipInside = false;

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Ship"))
        {
            ShipInside = true;
            Debug.Log("[DockGoal] Ship entered docking zone");
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Ship"))
        {
            ShipInside = false;
            Debug.Log("[DockGoal] Ship exited docking zone");
        }
    }
}
