using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 로봇 "몸 전체 크기"로 벽·가구와 겹치는지 검사해서, 겹치면 바로 직전 위치로 되돌립니다.
/// → 어떤 스크립트로 움직이든(WASD, ROS teleop, Nav2 자율주행, 웨이포인트) 벽을 못 뚫습니다.
///
/// 붙이는 곳: 로봇 맨 위 오브젝트 (rosmentor 또는 turtlebot3_burger)
/// 벽/가구에는 충돌체가 있어야 합니다 (AutoWallCollider 또는 FBX의 Generate Colliders).
///
/// Scene 화면에서 로봇을 선택하면 검사 상자가 초록색(통과)/빨간색(막힘)으로 보입니다.
/// </summary>
[DefaultExecutionOrder(1000)]   // 다른 이동 스크립트가 움직인 "다음"에 검사
public class RobotBodyBlocker : MonoBehaviour
{
    [Tooltip("바닥과 겹치지 않게 상자 아랫면을 이만큼 띄움 (m). 문턱 같은 낮은 턱은 넘어감")]
    public float floorClearance = 0.03f;
    [Tooltip("로봇 몸보다 상자를 이만큼 더 크게 (m). 벽에 너무 붙으면 늘리기")]
    public float padding = 0.01f;
    public LayerMask obstacleLayers = ~0;
    [Tooltip("막힐 때 무엇에 막혔는지 Console에 표시 (같은 물체는 한 번만)")]
    public bool logBlocker = true;

    Vector3 localCenter, halfExtents;
    Vector3 safePos; Quaternion safeRot;
    ArticulationBody rootAB;
    Rigidbody rb;
    readonly Collider[] hits = new Collider[64];
    readonly HashSet<Collider> ignore = new HashSet<Collider>();
    bool ready, blockedNow;
    string lastBlocker;

    void Start()
    {
        foreach (var ab in GetComponentsInChildren<ArticulationBody>())
            if (ab.isRoot) { rootAB = ab; break; }
        rb = GetComponent<Rigidbody>();

        ready = ComputeBox();
        if (!ready) { Debug.LogWarning("[RobotBodyBlocker] 로봇 모양(MeshFilter)을 못 찾아서 꺼집니다."); return; }

        // 시작할 때부터 겹쳐 있는 물체(바닥, 러그 등)는 무시 — 안 그러면 로봇이 처음부터 꼼짝 못 함
        foreach (var c in Overlaps())
        {
            ignore.Add(c);
            Debug.LogWarning($"[RobotBodyBlocker] 시작할 때부터 로봇과 겹친 '{c.name}'은(는) 검사에서 뺍니다. " +
                             "벽이라면 로봇을 벽에서 조금 떨어뜨려 놓고 다시 Play 하세요.");
        }
        safePos = transform.position; safeRot = transform.rotation;
    }

    void FixedUpdate()
    {
        if (!ready) return;

        Collider blocker = null;
        foreach (var c in Overlaps()) { blocker = c; break; }

        if (blocker == null)
        {
            safePos = transform.position; safeRot = transform.rotation;
            blockedNow = false;
            return;
        }

        blockedNow = true;
        if (logBlocker && blocker.name != lastBlocker)
        {
            lastBlocker = blocker.name;
            Debug.Log($"[RobotBodyBlocker] '{blocker.name}'에 막혀서 멈춤 (Hierarchy에서 노랗게 표시)", blocker);
#if UNITY_EDITOR
            UnityEditor.EditorGUIUtility.PingObject(blocker.gameObject);
#endif
        }

        // 직전의 안전한 위치로 되돌리기
        if (rootAB != null) rootAB.TeleportRoot(safePos, safeRot);
        else
        {
            transform.SetPositionAndRotation(safePos, safeRot);
            if (rb != null) { rb.position = safePos; rb.rotation = safeRot; }
        }
    }

    IEnumerable<Collider> Overlaps()
    {
        Vector3 center = transform.TransformPoint(localCenter);
        Vector3 s = transform.lossyScale;
        Vector3 ext = new Vector3(Mathf.Abs(halfExtents.x * s.x), Mathf.Abs(halfExtents.y * s.y), Mathf.Abs(halfExtents.z * s.z));
        int n = Physics.OverlapBoxNonAlloc(center, ext, hits, transform.rotation, obstacleLayers, QueryTriggerInteraction.Ignore);
        for (int i = 0; i < n; i++)
        {
            var c = hits[i];
            if (c == null || c.transform.IsChildOf(transform) || ignore.Contains(c)) continue;  // 자기 몸 제외
            yield return c;
        }
    }

    // 눈에 보이는 로봇 부품 전체를 감싸는 상자를 로봇 기준 좌표로 계산
    bool ComputeBox()
    {
        bool any = false;
        Vector3 min = Vector3.one * float.MaxValue, max = -min;
        Matrix4x4 toLocal = transform.worldToLocalMatrix;
        foreach (var mf in GetComponentsInChildren<MeshFilter>())
        {
            var r = mf.GetComponent<Renderer>();
            if (mf.sharedMesh == null || r == null || !r.enabled) continue;
            Bounds b = mf.sharedMesh.bounds;
            Matrix4x4 m = toLocal * mf.transform.localToWorldMatrix;
            for (int i = 0; i < 8; i++)
            {
                Vector3 corner = b.center + Vector3.Scale(b.extents,
                    new Vector3((i & 1) == 0 ? -1 : 1, (i & 2) == 0 ? -1 : 1, (i & 4) == 0 ? -1 : 1));
                Vector3 p = m.MultiplyPoint3x4(corner);
                min = Vector3.Min(min, p); max = Vector3.Max(max, p);
                any = true;
            }
        }
        if (!any) return false;

        float sy = Mathf.Max(Mathf.Abs(transform.lossyScale.y), 1e-6f);
        min.y += floorClearance / sy;
        if (max.y < min.y + 0.05f / sy) max.y = min.y + 0.05f / sy;
        localCenter = (min + max) * 0.5f;
        halfExtents = (max - min) * 0.5f + Vector3.one * (padding / sy);
        return true;
    }

    void OnDrawGizmosSelected()
    {
        if (!Application.isPlaying && !ComputeBox()) return;
        Gizmos.color = blockedNow ? Color.red : Color.green;
        Gizmos.matrix = Matrix4x4.TRS(transform.position, transform.rotation, transform.lossyScale);
        Gizmos.DrawWireCube(localCenter, halfExtents * 2f);
    }
}
