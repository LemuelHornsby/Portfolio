using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

#region JSON Types

[Serializable]
public class ScenarioRootJson
{
    public List<MarinaScenarioJson> marinas = new List<MarinaScenarioJson>();
}

[Serializable]
public class MarinaScenarioJson
{
    public int id;
    public string name;

    public List<ObstacleJson> static_obstacles = new List<ObstacleJson>();
    public List<WallBoxJson> walls = new List<WallBoxJson>();
    public List<DockTargetJson> dock_targets = new List<DockTargetJson>();
}

[Serializable]
public class ObstacleJson
{
    public int id;
    public string type;   // "buoy" / "lifebuoy" / "ship" / etc.
    public float x;       // fallback center x (Unity world X)
    public float y;       // fallback center y (Unity world Z)
    public float r;       // fallback radius (circle) OR capsule radius
    public bool dynamic;

    // --- NEW: footprint fields (XZ plane) ---
    public string shape;  // "circle" | "obb" | "capsule"

    // OBB/Capsule center (Unity world XZ)
    public float cx;
    public float cy;

    // OBB sizes (meters) along OBB local axes in XZ
    public float sx;
    public float sy;

    // yaw about +Y (radians)
    public float yaw;

    // Capsule segment length (meters). Capsule radius uses 'r'
    public float seg;
}

[Serializable]
public class WallBoxJson
{
    public int id;
    public float cx;  // center x
    public float cy;  // center z -> y
    public float sx;  // size along local X in world meters
    public float sy;  // size along local Z in world meters
    public float yaw; // radians about +Y
}

[Serializable]
public class DockTargetJson
{
    public int id;
    public float x;
    public float y;
    public float psi; // radians
    public string name; // optional
}

#endregion

public class MarinaScenarioExporter : MonoBehaviour
{
    [Header("Active marinas to export (use your active marinas here)")]
    public List<Transform> marinaRoots = new List<Transform>();

    [Header("Include traffic ships within this radius from each marina root (meters)")]
    public float includeRadius = 250f;

    [Header("Child roots inside each marina")]
    public string wallsRootName = "WallsRoot";
    public string dockGoalsActiveName = "DockGoals_Active";

    [Header("Traffic roots (ships live here, not under marinas)")]
    public Transform trafficStaticRoot;   // Traffic_Static_Root
    public Transform trafficDynamicRoot;  // Traffic_Dynamic_Root

    [Header("Output")]
    public string fileName = "marinas_export.json";

    [Header("Footprint settings")]
    [Tooltip("If true, ships export as capsule; otherwise ships export as OBB like others.")]
    public bool exportShipsAsCapsules = true;

    [Tooltip("Minimum footprint size to accept (meters). If smaller, fallback to safety radius.")]
    public float minFootprintSize = 0.05f;

    [ContextMenu("Export Marina Scenarios JSON")]
    public void Export()
    {
        var root = new ScenarioRootJson();

        for (int mi = 0; mi < marinaRoots.Count; mi++)
        {
            var mRoot = marinaRoots[mi];
            if (mRoot == null) continue;

            var mj = new MarinaScenarioJson
            {
                id = mi,
                name = mRoot.name
            };

            // 1) Export ACTIVE dock goals only (DockGoals_Active)
            ExportDockGoalsActive(mj, mRoot);

            // 2) Export walls from BoxColliders under WallsRoot
            ExportWalls(mj, mRoot, mi);

            // 3) Export obstacles under marina (buoys etc.) that have MarinaObstacle
            ExportLocalMarinaObstacles(mj, mRoot);

            // 4) Export nearby traffic ships (static + dynamic), filtered by distance
            AddTrafficObstaclesNear(mj, trafficStaticRoot, mRoot.position, includeRadius, dynamicFlag: false);
            AddTrafficObstaclesNear(mj, trafficDynamicRoot, mRoot.position, includeRadius, dynamicFlag: true);

            root.marinas.Add(mj);
        }

        string json = JsonUtility.ToJson(root, true);
        string path = Path.Combine(Application.streamingAssetsPath, fileName);
        Directory.CreateDirectory(Application.streamingAssetsPath);
        File.WriteAllText(path, json);

        Debug.Log($"Exported marina scenarios to: {path}");
    }

    #region Dock Goals / Walls (unchanged)

    private void ExportDockGoalsActive(MarinaScenarioJson mj, Transform marinaRoot)
    {
        var dockRoot = marinaRoot.Find(dockGoalsActiveName);
        if (dockRoot == null)
        {
            Debug.LogWarning($"[{marinaRoot.name}] Could not find '{dockGoalsActiveName}'. No dock goals exported for this marina.");
            return;
        }

        for (int i = 0; i < dockRoot.childCount; i++)
        {
            var t = dockRoot.GetChild(i);
            int parsedId = TryParseDockGoalId(t.name, fallback: i);

            mj.dock_targets.Add(new DockTargetJson
            {
                id = parsedId,
                name = t.name,
                x = t.position.x,
                y = t.position.z, // Unity Z -> exported as Python y
                psi = t.eulerAngles.y * Mathf.Deg2Rad
            });
        }
    }

    private void ExportWalls(MarinaScenarioJson mj, Transform marinaRoot, int marinaIndex)
    {
        var wRoot = marinaRoot.Find(wallsRootName);
        if (wRoot == null)
        {
            Debug.LogWarning($"[{marinaRoot.name}] Could not find '{wallsRootName}'. No walls exported for this marina.");
            return;
        }

        var cols = wRoot.GetComponentsInChildren<BoxCollider>(true);
        int wid = 9000 + marinaIndex * 1000;

        foreach (var bc in cols)
        {
            Vector3 c = bc.transform.TransformPoint(bc.center);

            Vector3 s = bc.size;
            Vector3 ls = bc.transform.lossyScale;
            float sx = Mathf.Abs(s.x * ls.x);
            float sy = Mathf.Abs(s.z * ls.z);

            float yaw = bc.transform.eulerAngles.y * Mathf.Deg2Rad;

            mj.walls.Add(new WallBoxJson
            {
                id = wid++,
                cx = c.x,
                cy = c.z,
                sx = sx,
                sy = sy,
                yaw = yaw
            });
        }
    }

    #endregion

    #region Obstacles with geometry footprints

    private void ExportLocalMarinaObstacles(MarinaScenarioJson mj, Transform marinaRoot)
    {
        var localObs = marinaRoot.GetComponentsInChildren<MarinaObstacle>(true);
        foreach (var o in localObs)
        {
            var oj = BuildObstacleJsonFromGeometry(o, dynamicFlag: false);
            mj.static_obstacles.Add(oj);
        }
    }

    private void AddTrafficObstaclesNear(MarinaScenarioJson mj, Transform trafficRoot, Vector3 marinaPos, float radius, bool dynamicFlag)
    {
        if (trafficRoot == null) return;

        float r2 = radius * radius;

        var obs = trafficRoot.GetComponentsInChildren<MarinaObstacle>(true);
        foreach (var o in obs)
        {
            Vector3 p = o.transform.position;

            float dx = p.x - marinaPos.x;
            float dz = p.z - marinaPos.z;
            if ((dx * dx + dz * dz) > r2) continue;

            var oj = BuildObstacleJsonFromGeometry(o, dynamicFlag);
            mj.static_obstacles.Add(oj);
        }
    }

    private ObstacleJson BuildObstacleJsonFromGeometry(MarinaObstacle o, bool dynamicFlag)
    {
        string type = o.kind.ToString().ToLower();

        // Fallback circle (always valid)
        var oj = new ObstacleJson
        {
            id = o.id,
            type = type,
            dynamic = dynamicFlag,
            shape = "circle",
            x = o.transform.position.x,
            y = o.transform.position.z,
            r = o.safetyRadius
        };

        // Try geometry OBB from renderers (pivot/colliders not required)
        if (TryComputeObbFromRenderersXZ(o.transform, out float cx, out float cy, out float sx, out float sy, out float yawRad))
        {
            // Reject tiny/degenerate footprints
            if (sx >= minFootprintSize && sy >= minFootprintSize)
            {
                oj.cx = cx; oj.cy = cy;
                oj.sx = sx; oj.sy = sy;
                oj.yaw = yawRad;

                // Also set fallback circle centered at footprint (helpful for older Python)
                oj.x = cx; oj.y = cy;
                oj.r = 0.5f * Mathf.Sqrt(sx * sx + sy * sy);

                // Ships: export capsule derived from OBB (optional)
                if (exportShipsAsCapsules && type == "ship")
                {
                    ObbToCapsule(sx, sy, out float radius, out float seg);
                    oj.shape = "capsule";
                    oj.r = radius;   // capsule radius
                    oj.seg = seg;    // capsule straight segment length
                }
                else
                {
                    oj.shape = "obb";
                }
            }
        }

        return oj;
    }

    /// <summary>
    /// Fit an OBB in the XZ plane from Renderer bounds (fast, robust).
    /// No colliders and no reliable pivot needed.
    /// Returns center (cx,cy) in world XZ, size (sx,sy) in meters, yaw in radians about +Y.
    /// </summary>
    private bool TryComputeObbFromRenderersXZ(
        Transform root,
        out float cx, out float cy,
        out float sx, out float sy,
        out float yawRad)
    {
        cx = cy = sx = sy = yawRad = 0f;

        var rends = root.GetComponentsInChildren<Renderer>(true);
        if (rends == null || rends.Length == 0) return false;

        List<Vector2> pts = new List<Vector2>(rends.Length * 8);

        foreach (var r in rends)
        {
            if (r == null) continue;
            Bounds b = r.bounds;
            Vector3 c = b.center;
            Vector3 e = b.extents;

            // 8 corners of the bounds AABB
            for (int xi = -1; xi <= 1; xi += 2)
            for (int yi = -1; yi <= 1; yi += 2)
            for (int zi = -1; zi <= 1; zi += 2)
            {
                Vector3 w = c + Vector3.Scale(e, new Vector3(xi, yi, zi));
                pts.Add(new Vector2(w.x, w.z)); // XZ plane
            }
        }

        if (pts.Count < 3) return false;

        // Mean
        Vector2 mean = Vector2.zero;
        for (int i = 0; i < pts.Count; i++) mean += pts[i];
        mean /= pts.Count;

        // Covariance
        float cxx = 0f, cxy = 0f, cyy = 0f;
        for (int i = 0; i < pts.Count; i++)
        {
            Vector2 d = pts[i] - mean;
            cxx += d.x * d.x;
            cxy += d.x * d.y;
            cyy += d.y * d.y;
        }
        cxx /= pts.Count; cxy /= pts.Count; cyy /= pts.Count;

        // PCA major axis angle: tan(2a)=2cxy/(cxx-cyy)
        float angle = 0.5f * Mathf.Atan2(2f * cxy, (cxx - cyy));
        float ca = Mathf.Cos(angle), sa = Mathf.Sin(angle);

        // Bounds in PCA frame
        float minU = float.PositiveInfinity, maxU = float.NegativeInfinity;
        float minV = float.PositiveInfinity, maxV = float.NegativeInfinity;

        for (int i = 0; i < pts.Count; i++)
        {
            Vector2 d = pts[i] - mean;

            // [u; v] = R^T d  where R = [[ca,-sa],[sa,ca]]
            float u =  ca * d.x + sa * d.y;
            float v = -sa * d.x + ca * d.y;

            if (u < minU) minU = u;
            if (u > maxU) maxU = u;
            if (v < minV) minV = v;
            if (v > maxV) maxV = v;
        }

        float du = maxU - minU;
        float dv = maxV - minV;
        if (du < minFootprintSize || dv < minFootprintSize) return false;

        // Center in PCA frame then rotate back
        float cu = 0.5f * (minU + maxU);
        float cv = 0.5f * (minV + maxV);

        float worldX = mean.x + (ca * cu - sa * cv);
        float worldY = mean.y + (sa * cu + ca * cv);

        cx = worldX;
        cy = worldY;
        sx = du;
        sy = dv;
        yawRad = angle;
        return true;
    }

    private void ObbToCapsule(float sx, float sy, out float radius, out float seg)
    {
        float longDim = Mathf.Max(sx, sy);
        float shortDim = Mathf.Min(sx, sy);
        radius = 0.5f * shortDim;
        seg = Mathf.Max(0f, longDim - 2f * radius);
    }

    #endregion

    #region DockGoal ID parsing (unchanged)

    private int TryParseDockGoalId(string name, int fallback)
    {
        if (string.IsNullOrEmpty(name)) return fallback;

        int idx = name.IndexOf("DockGoal_", StringComparison.OrdinalIgnoreCase);
        if (idx < 0) return fallback;

        idx += "DockGoal_".Length;
        if (idx >= name.Length) return fallback;

        int end = idx;
        while (end < name.Length && char.IsDigit(name[end])) end++;

        if (end == idx) return fallback;

        string numStr = name.Substring(idx, end - idx);
        if (int.TryParse(numStr, out int id))
            return id;

        return fallback;
    }

    #endregion
}