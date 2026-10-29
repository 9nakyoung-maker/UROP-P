#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;

/// <summary>
/// 로봇 크기를 실물 치수(mm)에 맞추는 창.
/// 넣는 곳: Assets/Editor/RobotSizeMatcher.cs
/// 사용: Tools → 로봇 실물 크기 맞추기 → [지금 크기 재기] → [실물 크기로 맞추기] → Ctrl+S
///
/// 기본값은 Hiwonder LanderPi 메카넘(깊이카메라 버전) 공식 치수: 212 × 174 × 441 mm, 약 1.79kg
/// 높이는 팔 자세에 따라 달라지므로, 비율은 길이·폭(바닥 크기)으로 계산합니다.
/// </summary>
public class RobotSizeMatcher : EditorWindow
{
    GameObject robot;
    float realLength = 212f, realWidth = 174f, realHeight = 441f;   // mm
    Vector3 measured;   // Unity에서 잰 길이(앞뒤) / 폭(좌우) / 높이, m
    bool hasMeasure;

    [MenuItem("Tools/로봇 실물 크기 맞추기")]
    static void Open() => GetWindow<RobotSizeMatcher>("로봇 실물 크기");

    void OnEnable() { if (robot == null) robot = GameObject.Find("rosmentor"); }

    void OnGUI()
    {
        robot = (GameObject)EditorGUILayout.ObjectField("로봇 (맨 위 오브젝트)", robot, typeof(GameObject), true);
        EditorGUILayout.Space();
        EditorGUILayout.LabelField("실물 치수 (mm)", EditorStyles.boldLabel);
        realLength = EditorGUILayout.FloatField("길이 (앞뒤)", realLength);
        realWidth = EditorGUILayout.FloatField("폭 (좌우)", realWidth);
        realHeight = EditorGUILayout.FloatField("높이 (참고용)", realHeight);
        EditorGUILayout.HelpBox("기본값: LanderPi 메카넘 212 × 174 × 441 mm. 자로 직접 잰 값이 있으면 그걸 넣으세요.", MessageType.None);

        if (robot == null) { EditorGUILayout.HelpBox("Hierarchy에서 rosmentor를 위 칸에 끌어다 놓으세요.", MessageType.Warning); return; }

        EditorGUILayout.Space();
        if (GUILayout.Button("📏 지금 크기 재기", GUILayout.Height(28))) { hasMeasure = Measure(out measured); }

        if (hasMeasure)
        {
            float s = Mathf.Abs(robot.transform.localScale.x);
            EditorGUILayout.LabelField("Unity 속 크기 (mm)", EditorStyles.boldLabel);
            EditorGUILayout.LabelField($"길이 {measured.z * 1000f:0}   폭 {measured.x * 1000f:0}   높이 {measured.y * 1000f:0}");
            float ratio = Ratio();
            EditorGUILayout.LabelField($"실물 대비: {1f / ratio * 100f:0}%   (지금 Scale {s:0.###})");
            EditorGUILayout.LabelField($"→ 맞추려면 Scale = {s * ratio:0.###}", EditorStyles.boldLabel);

            if (GUILayout.Button("✅ 실물 크기로 맞추기", GUILayout.Height(28)))
            {
                Undo.RecordObject(robot.transform, "Robot real size");
                robot.transform.localScale = Vector3.one * (s * ratio);
                UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(robot.scene);
                hasMeasure = Measure(out measured);
                Debug.Log($"[RobotSizeMatcher] {robot.name} Scale → {robot.transform.localScale.x:0.###} (Ctrl+S로 저장, Ctrl+Z로 되돌리기)");
            }
        }
    }

    // 길이·폭 두 방향 비율의 평균 (팔 자세에 영향받는 높이는 제외)
    float Ratio()
    {
        float rl = measured.z > 1e-4f ? (realLength / 1000f) / measured.z : 1f;
        float rw = measured.x > 1e-4f ? (realWidth / 1000f) / measured.x : 1f;
        return (rl + rw) * 0.5f;
    }

    // 로봇 기준(앞=+Z, 왼오=X, 위=Y)으로 눈에 보이는 부품 전체를 감싸는 크기 (월드 m)
    bool Measure(out Vector3 size)
    {
        size = Vector3.zero;
        Transform root = robot.transform;
        Vector3 min = Vector3.one * float.MaxValue, max = -min;
        bool any = false;
        Matrix4x4 toLocal = Matrix4x4.TRS(root.position, root.rotation, Vector3.one).inverse;   // 회전만 풀고 크기는 월드 그대로
        foreach (var mf in robot.GetComponentsInChildren<MeshFilter>())
        {
            var r = mf.GetComponent<Renderer>();
            if (mf.sharedMesh == null || r == null || !r.enabled || IsArm(mf.transform, root)) continue;
            Bounds b = mf.sharedMesh.bounds;
            Matrix4x4 m = toLocal * mf.transform.localToWorldMatrix;
            for (int i = 0; i < 8; i++)
            {
                Vector3 p = m.MultiplyPoint3x4(b.center + Vector3.Scale(b.extents,
                    new Vector3((i & 1) == 0 ? -1 : 1, (i & 2) == 0 ? -1 : 1, (i & 4) == 0 ? -1 : 1)));
                min = Vector3.Min(min, p); max = Vector3.Max(max, p); any = true;
            }
        }
        if (any) size = max - min;
        return any;
    }

    // 팔(link1~5)·그리퍼는 자세에 따라 앞으로 튀어나와 길이를 틀리게 만들므로 빼고 잼 (차체 크기만)
    static bool IsArm(Transform t, Transform root)
    {
        for (; t != null && t != root; t = t.parent)
        {
            string n = t.name.ToLowerInvariant();
            if (n.StartsWith("link") || n.Contains("gripper") || n.Contains("jaw")) return true;
        }
        return false;
    }
}
#endif
