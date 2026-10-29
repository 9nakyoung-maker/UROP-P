#if UNITY_EDITOR
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

/// <summary>
/// URDF 로봇의 "눈에 보이는 모양(Visuals)"이 실제 링크 위치(Collisions)와 어긋났을 때 한 번에 맞춰 줍니다.
/// 각 링크마다 Visuals 묶음을 같은 링크의 Collisions 묶음 위치로 옮깁니다. (둘은 같은 STL 모양이라 겹쳐야 정상)
///
/// 넣는 곳: Assets/Editor/RobotVisualAligner.cs   (Editor 폴더 안)
/// 사용: Hierarchy에서 로봇(rosmentor) 클릭 → 메뉴 Tools → 로봇 모양 위치 맞추기 → Ctrl+S
///       잘못되면 Ctrl+Z 한 번으로 전부 되돌아갑니다.
/// </summary>
public static class RobotVisualAligner
{
    const float MinShift = 0.02f;   // 2cm 이하로 어긋난 건 그대로 둠

    [MenuItem("Tools/로봇 모양 위치 맞추기")]
    static void Align()
    {
        GameObject robot = Selection.activeGameObject;
        if (robot == null || robot.GetComponentInChildren<Transform>() == null || FindChildren(robot.transform, "Visuals").Count == 0)
            robot = GameObject.Find("rosmentor");
        if (robot == null)
        {
            EditorUtility.DisplayDialog("로봇 모양 위치 맞추기", "Hierarchy에서 로봇(rosmentor)을 먼저 클릭하세요.", "확인");
            return;
        }
        // 선택한 게 로봇 안쪽 부품이면 맨 위 로봇까지 올라감
        while (robot.transform.parent != null && robot.GetComponent("UrdfRobot") == null) robot = robot.transform.parent.gameObject;

        int group = Undo.GetCurrentGroup();
        var log = new List<string>();
        foreach (Transform vis in FindChildren(robot.transform, "Visuals"))
        {
            Transform col = vis.parent != null ? vis.parent.Find("Collisions") : null;
            if (col == null) continue;
            if (!VisualCenter(vis, out Vector3 vc) || !CollisionCenter(col, out Vector3 cc)) continue;

            Vector3 delta = cc - vc;
            if (delta.magnitude < MinShift) continue;
            Undo.RecordObject(vis, "Align robot visuals");
            vis.position += delta;
            log.Add($"{vis.parent.name}: {delta.magnitude * 100f:0}cm 이동");
        }
        Undo.CollapseUndoOperations(group);
        UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(robot.scene);

        string msg = log.Count == 0
            ? "어긋난 부품이 없어요 (모두 2cm 이내). 다른 원인일 수 있어요."
            : $"{log.Count}개 링크의 모양을 제자리로 옮겼어요.\n\n• " + string.Join("\n• ", log.GetRange(0, Mathf.Min(15, log.Count)))
              + (log.Count > 15 ? "\n• ..." : "") + "\n\n괜찮으면 Ctrl+S, 이상하면 Ctrl+Z";
        Debug.Log("[RobotVisualAligner] " + msg.Replace("\n", " "));
        EditorUtility.DisplayDialog("로봇 모양 위치 맞추기", msg, "확인");
    }

    static List<Transform> FindChildren(Transform root, string name)
    {
        var list = new List<Transform>();
        foreach (var t in root.GetComponentsInChildren<Transform>(true)) if (t.name == name) list.Add(t);
        return list;
    }

    static bool VisualCenter(Transform vis, out Vector3 center)
    {
        var b = new Bounds(); bool any = false;
        foreach (var mf in vis.GetComponentsInChildren<MeshFilter>(true))
            if (mf.sharedMesh != null) Encapsulate(ref b, ref any, mf.sharedMesh.bounds, mf.transform.localToWorldMatrix);
        center = b.center; return any;
    }

    static bool CollisionCenter(Transform col, out Vector3 center)
    {
        var b = new Bounds(); bool any = false;
        foreach (var c in col.GetComponentsInChildren<Collider>(true))
        {
            Matrix4x4 m = c.transform.localToWorldMatrix;
            if (c is MeshCollider mc && mc.sharedMesh != null) Encapsulate(ref b, ref any, mc.sharedMesh.bounds, m);
            else if (c is BoxCollider bc) Encapsulate(ref b, ref any, new Bounds(bc.center, bc.size), m);
            else if (c is SphereCollider sc) Encapsulate(ref b, ref any, new Bounds(sc.center, Vector3.one * sc.radius * 2f), m);
            else if (c is CapsuleCollider cc) Encapsulate(ref b, ref any, new Bounds(cc.center, Vector3.one * Mathf.Max(cc.radius * 2f, cc.height)), m);
        }
        center = b.center; return any;
    }

    static void Encapsulate(ref Bounds b, ref bool any, Bounds local, Matrix4x4 m)
    {
        for (int i = 0; i < 8; i++)
        {
            Vector3 p = m.MultiplyPoint3x4(local.center + Vector3.Scale(local.extents,
                new Vector3((i & 1) == 0 ? -1 : 1, (i & 2) == 0 ? -1 : 1, (i & 4) == 0 ? -1 : 1)));
            if (!any) { b = new Bounds(p, Vector3.zero); any = true; } else b.Encapsulate(p);
        }
    }
}
#endif
