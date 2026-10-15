using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class NodeData
{
    public string Name;
    public Transform WayPoint;
    public List<NodeData> Neighbors = new List<NodeData>();

    [HideInInspector] public float gScore = float.MaxValue;
    [HideInInspector] public NodeData parent = null;
}

public class MyLineFollower : MonoBehaviour
{
    [Header("전체 경로 점들 (0번이 Point_0 / Home)")]
    public List<NodeData> Nodes = new List<NodeData>();

    [Header("이동 속도 및 설정")]
    public float MoveSpeed = 2.0f;
    public float RotateSpeed = 5.0f;
    public float ArrivalDistance = 0.2f;

    [Header("목적지 인덱스 번호")]
    public int AirconNodeIndex = 3;      // A 키 (Point_3)
    public int LightSwitchNodeIndex = 6; // B 키 (Point_6)
    public int HomeNodeIndex = 0;        // C 키 (Point_0)

    [Header("선 바닥 띄움 높이")]
    public float LineOffsetY = 0.01f;     // 바닥에 파묻히지 않게 적절히 띄움

    private LineRenderer lineRenderer;
    private List<Transform> currentPath = new List<Transform>();
    private int currentPathIndex = 0;
    private bool isMoving = false;

    void Start()
    {
        lineRenderer = GetComponent<LineRenderer>();
        if (lineRenderer == null)
        {
            lineRenderer = gameObject.AddComponent<LineRenderer>();
        }

        // 실행 직후 잔여 선 지우기
        if (lineRenderer != null) lineRenderer.positionCount = 0;

        // 인스펙터 참조 자동 연결 (다익스트라 탐색용)
        LinkNodeNeighbors();

        // 로봇 시작 위치를 Point_0 (Home) 위치에 정렬
        if (Nodes != null && Nodes.Count > 0 && Nodes[HomeNodeIndex].WayPoint != null)
        {
            Vector3 startPos = Nodes[HomeNodeIndex].WayPoint.position;
            transform.position = new Vector3(startPos.x, transform.position.y, startPos.z);
        }
    }

    private void LinkNodeNeighbors()
    {
        Dictionary<Transform, NodeData> nodeMap = new Dictionary<Transform, NodeData>();
        foreach (var node in Nodes)
        {
            if (node != null && node.WayPoint != null)
            {
                if (!nodeMap.ContainsKey(node.WayPoint))
                    nodeMap.Add(node.WayPoint, node);
            }
        }

        foreach (var node in Nodes)
        {
            if (node == null) continue;
            List<NodeData> realNeighbors = new List<NodeData>();
            foreach (var neighbor in node.Neighbors)
            {
                if (neighbor != null && neighbor.WayPoint != null && nodeMap.ContainsKey(neighbor.WayPoint))
                {
                    realNeighbors.Add(nodeMap[neighbor.WayPoint]);
                }
            }
            node.Neighbors = realNeighbors;
        }
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.A))
        {
            SetDestination(AirconNodeIndex);
        }
        else if (Input.GetKeyDown(KeyCode.B))
        {
            SetDestination(LightSwitchNodeIndex);
        }
        else if (Input.GetKeyDown(KeyCode.C))
        {
            SetDestination(HomeNodeIndex);
        }

        if (isMoving && currentPath != null && currentPath.Count > 0)
        {
            MoveAlongPath();
        }
    }

    public void SetDestination(int targetIndex)
    {
        if (Nodes == null || Nodes.Count == 0 || targetIndex < 0 || targetIndex >= Nodes.Count)
        {
            Debug.LogError("Nodes 목록이 비어있거나 인덱스가 올바르지 않습니다.");
            return;
        }

        NodeData startNode = GetClosestNode(transform.position);
        NodeData targetNode = Nodes[targetIndex];

        if (startNode == null || targetNode == null)
        {
            Debug.LogError("시작 또는 목적지 노드를 찾지 못했습니다.");
            return;
        }

        // 1. 다익스트라 최단 경로 계산
        currentPath = CalculateDijkstraPath(startNode, targetNode);

        if (currentPath != null && currentPath.Count > 0)
        {
            currentPathIndex = 0;
            isMoving = true;

            // 2. [가시화] 경로 계산 완료 즉시 전체 빨간색 경로 선을 바닥에 먼저 그림!
            DrawFullPathLine();

            string pathStr = "";
            foreach (var t in currentPath) pathStr += t.name + " ➔ ";
            Debug.Log($"[경로 탐색 성공] 전체 경로 가시화 완료: {pathStr}");
        }
        else
        {
            Debug.LogWarning("경로 탐색 실패: 연결 노드를 확인하세요.");
        }
    }

    // 전체 경로 선을 바닥에 미리 그려주는 함수
    private void DrawFullPathLine()
    {
        if (lineRenderer == null || currentPath == null || currentPath.Count == 0) return;

        // 시작점(현재 로봇 위치) + 경로 상의 노드들
        lineRenderer.positionCount = currentPath.Count + 1;

        // transform.position.y 대신 WayPoint 바닥 높이(또는 0.01f 등 바닥 고정값) 기준 적용
        float floorY = currentPath[0].position.y + LineOffsetY;

        // 0번 포인트: 로봇의 출발 위치
        lineRenderer.SetPosition(0, new Vector3(transform.position.x, floorY, transform.position.z));

        // 1번 포인트부터 끝까지: 경로 상의 모든 WayPoint 위치
        for (int i = 0; i < currentPath.Count; i++)
        {
            Vector3 pPos = currentPath[i].position;
            lineRenderer.SetPosition(i + 1, new Vector3(pPos.x, floorY, pPos.z));
        }
    }

    private NodeData GetClosestNode(Vector3 pos)
    {
        NodeData closest = null;
        float minDist = float.MaxValue;

        foreach (var node in Nodes)
        {
            if (node == null || node.WayPoint == null) continue;

            float dist = Vector2.Distance(new Vector2(pos.x, pos.z), new Vector2(node.WayPoint.position.x, node.WayPoint.position.z));
            if (dist < minDist)
            {
                minDist = dist;
                closest = node;
            }
        }
        return closest;
    }

    private List<Transform> CalculateDijkstraPath(NodeData start, NodeData target)
    {
        foreach (var n in Nodes)
        {
            if (n == null) continue;
            n.gScore = float.MaxValue;
            n.parent = null;
        }

        List<NodeData> openList = new List<NodeData>();
        start.gScore = 0;
        openList.Add(start);

        while (openList.Count > 0)
        {
            openList.Sort((a, b) => a.gScore.CompareTo(b.gScore));
            NodeData current = openList[0];
            openList.RemoveAt(0);

            if (current == target)
            {
                List<Transform> path = new List<Transform>();
                NodeData curr = target;
                while (curr != null)
                {
                    if (curr.WayPoint != null) path.Add(curr.WayPoint);
                    curr = curr.parent;
                }
                path.Reverse();
                return path;
            }

            foreach (var neighbor in current.Neighbors)
            {
                if (neighbor == null || neighbor.WayPoint == null) continue;

                float dist = Vector3.Distance(current.WayPoint.position, neighbor.WayPoint.position);
                float newScore = current.gScore + dist;

                if (newScore < neighbor.gScore)
                {
                    neighbor.gScore = newScore;
                    neighbor.parent = current;
                    if (!openList.Contains(neighbor)) openList.Add(neighbor);
                }
            }
        }
        return null;
    }

    private void MoveAlongPath()
    {
        if (currentPathIndex >= currentPath.Count)
        {
            isMoving = false;
            // 목적지 도착 시 선 지우기 (원치 않으면 이 주석 줄을 지우세요)
            // if (lineRenderer != null) lineRenderer.positionCount = 0;
            Debug.Log("목적지에 무사히 도착했습니다!");
            return;
        }

        Transform targetWP = currentPath[currentPathIndex];
        Vector3 targetPos = new Vector3(targetWP.position.x, transform.position.y, targetWP.position.z);

        Vector3 dir = (targetPos - transform.position).normalized;
        if (dir != Vector3.zero)
        {
            Quaternion targetRot = Quaternion.LookRotation(dir);
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRot, RotateSpeed * Time.deltaTime);
        }

        transform.position = Vector3.MoveTowards(transform.position, targetPos, MoveSpeed * Time.deltaTime);

        if (Vector2.Distance(new Vector2(transform.position.x, transform.position.z), new Vector2(targetPos.x, targetPos.z)) < ArrivalDistance)
        {
            currentPathIndex++;
        }
    }

    private void OnDrawGizmos()
    {
        if (Nodes == null || Nodes.Count == 0) return;

        foreach (var node in Nodes)
        {
            if (node == null || node.WayPoint == null) continue;

            Vector3 nodePos = node.WayPoint.position;

            if (node.Neighbors != null)
            {
                Gizmos.color = Color.red;
                foreach (var neighbor in node.Neighbors)
                {
                    if (neighbor != null && neighbor.WayPoint != null)
                    {
                        Gizmos.DrawLine(nodePos, neighbor.WayPoint.position);
                    }
                }
            }
        }
    }
}
