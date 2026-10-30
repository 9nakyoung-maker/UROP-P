using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

/// <summary>
/// 이 오브젝트 "아래에 들어있는" 모든 구조물(벽, 책상, 에어컨...)에 자동으로 충돌체(MeshCollider)를 붙여서
/// 로봇이 뚫고 지나가지 못하게 하고, LiDAR에도 벽으로 보이게 합니다.
///
/// 붙이는 곳: 방/구조물을 모아둔 "부모" 오브젝트 (예: Room, Environment)
/// 새 구조물은 Hierarchy에서 이 부모 오브젝트 밑으로 끌어다 넣기만 하면 됩니다.
///
///  - Play를 누르면 자동 적용 (Play 중에 새로 넣은 것도 1초 안에 적용)
///  - Play 안 누르고 영구 적용: Inspector에서 이 스크립트 ⋮ 메뉴 → "지금 콜라이더 붙이기" → Ctrl+S
/// </summary>
[DefaultExecutionOrder(-100)]
public class AutoWallCollider : MonoBehaviour
{
    [Tooltip("Play 중에 새로 추가된 구조물도 자동으로 처리")]
    public bool watchNewChildren = true;

    [Tooltip("이 이름의 오브젝트(와 그 아래)는 건드리지 않음 — 로봇이 실수로 이 아래 들어가 있어도 안전")]
    public string[] skipNames = { "rosmentor", "turtlebot3_burger", "RealRobotLink" };

    const int IgnoreRaycastLayer = 2;
    float next;
    int lastCount = -1;

    void Awake() => Apply();

    void Update()
    {
        if (!watchNewChildren || Time.time < next) return;
        next = Time.time + 1f;
        int count = GetComponentsInChildren<Transform>(true).Length;
        if (count != lastCount) Apply();
    }

    [ContextMenu("지금 콜라이더 붙이기")]
    public void Apply()
    {
        int added = 0, layerFixed = 0;
        foreach (var mf in GetComponentsInChildren<MeshFilter>(true))
        {
            if (mf.sharedMesh == null || IsSkipped(mf.transform)) continue;

            // 화면에 안 보이는(렌더러가 없거나 꺼진) 메시는 "투명 벽"이 되므로 건너뜀
            var rend = mf.GetComponent<Renderer>();
            if (rend == null || !rend.enabled) continue;

            GameObject go = mf.gameObject;

            // Ignore Raycast 레이어면 LiDAR가 못 봐서 일반 레이어로 바꿔줌 (로봇 몸만 Ignore Raycast여야 함)
            if (go.layer == IgnoreRaycastLayer)
            {
#if UNITY_EDITOR
                if (!Application.isPlaying) Undo.RecordObject(go, "AutoWallCollider");
#endif
                go.layer = 0;
                layerFixed++;
            }

            if (HasSolidCollider(go)) continue;   // 이미 (켜져 있고, Trigger가 아닌) 충돌체가 있으면 그대로 둠

            MeshCollider mc;
#if UNITY_EDITOR
            if (!Application.isPlaying) mc = Undo.AddComponent<MeshCollider>(go);
            else
#endif
            mc = go.AddComponent<MeshCollider>();
            mc.sharedMesh = mf.sharedMesh;
            added++;
        }

        lastCount = GetComponentsInChildren<Transform>(true).Length;

#if UNITY_EDITOR
        if (!Application.isPlaying && (added > 0 || layerFixed > 0))
            UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(gameObject.scene);
#endif
        if (added > 0 || layerFixed > 0)
            Debug.Log($"[AutoWallCollider] 충돌체 {added}개 추가, 레이어 {layerFixed}개 수정 ({name} 아래)");
    }

    // 꺼진 충돌체나 Is Trigger 충돌체는 로봇을 못 막으므로 "없는 것"으로 취급
    static bool HasSolidCollider(GameObject go)
    {
        foreach (var c in go.GetComponents<Collider>())
            if (c.enabled && !c.isTrigger) return true;
        return false;
    }

    bool IsSkipped(Transform t)
    {
        for (; t != null && t != transform; t = t.parent)
            foreach (var n in skipNames)
                if (t.name == n) return true;
        return false;
    }
}
