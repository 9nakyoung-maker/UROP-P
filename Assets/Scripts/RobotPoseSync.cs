using System;
using System.Collections;
using UnityEngine;
using RosSharp.RosBridgeClient;
using Tf2 = RosSharp.RosBridgeClient.MessageTypes.Tf2;

// =====================================================================================
//  RobotPoseSync — 실제 로봇의 SLAM(AMCL) 좌표를 기준으로 Unity 로봇 위치를 실시간 동기화
//  붙이는 곳: RosConnector가 있는 오브젝트 (rosmentor)
// =====================================================================================
public class RobotPoseSync : MonoBehaviour
{
    [Header("필수 연결")]
    [Tooltip("비워두면 씬에서 자동으로 찾음")]
    public RosConnector rosConnector;

    [Tooltip("416MAP의 원점과 맞춰둔 MapOrigin 오브젝트")]
    public Transform mapOrigin;

    [Tooltip("실제 로봇 좌표대로 움직일 Unity 로봇 (비워두면 이 스크립트가 붙은 오브젝트)")]
    public Transform unityRobot;

    [Header("위치 및 방향 미세 보정")]
    [Tooltip("Unity 바닥 높이 (보통 0)")]
    public float floorHeight = 0f;

    [Tooltip("로봇 모델의 정면 방향이 어긋나면 90, -90, 180 등으로 보정")]
    public float yawOffset = 0f;

    [Header("시작 시 자동 위치 탐색")]
    [Tooltip("Play를 누르면 로봇에게 '방 전체에서 내 위치 찾기' 신호를 자동으로 보냄")]
    public bool autoLocateOnStart = true;

    [Header("ROS 좌표계 프레임 이름 (수정 불필요)")]
    public string tfTopic = "/tf";
    public string mapFrame = "map";
    public string odomFrame = "odom";
    public string baseFrame = "base_footprint";

    // 내부 좌표 계산용 변수
    private readonly object lockObj = new object();
    private bool hasMapToOdom = false;
    private bool hasOdomToBase = false;
    private float moX, moY, moYaw;
    private float obX, obY, obYaw;
    private bool isSynced = false;

    [Serializable]
    public class EmptyRequest : Message { }
    [Serializable]
    public class EmptyResponse : Message { }

    IEnumerator Start()
    {
        if (rosConnector == null) rosConnector = FindFirstObjectByType<RosConnector>();
        if (unityRobot == null) unityRobot = this.transform;

        if (rosConnector == null || mapOrigin == null)
        {
            Debug.LogError("[RobotPoseSync] RosConnector 또는 MapOrigin이 연결되지 않았습니다!");
            yield break;
        }

        // 1. 로봇(rosbridge)과 연결될 때까지 대기
        while (rosConnector.RosSocket == null || rosConnector.IsConnected == null || !rosConnector.IsConnected.WaitOne(0))
        {
            yield return new WaitForSeconds(0.5f);
        }

        // 2. 실제 로봇의 위치 좌표(/tf) 구독 시작
        rosConnector.RosSocket.Subscribe<Tf2.TFMessage>(tfTopic, OnTfReceived, 0, 10);
        Debug.Log("[RobotPoseSync] 로봇 연결 완료. 좌표 수신 대기 중...");

        // 3. 실행 즉시 방 전체에서 자기 위치를 찾도록 AMCL 초기화 신호 전송
        if (autoLocateOnStart)
        {
            yield return new WaitForSeconds(0.5f);
            RequestGlobalLocalization();
        }
    }

    // 로봇에 "전체 지도에서 내 위치를 스스로 찾아라" 명령 전송
    public void RequestGlobalLocalization()
    {
        if (rosConnector == null || rosConnector.RosSocket == null) return;

        rosConnector.RosSocket.CallService<EmptyRequest, EmptyResponse>(
            "/reinitialize_global_localization",
            (resp) =>
            {
                Debug.Log("[RobotPoseSync] 전체 지도 위치 탐색 시작! 로봇을 제자리에서 천천히 한 바퀴 회전시키면 위치가 정확히 잡힙니다.");
            },
            new EmptyRequest()
        );
    }

    // ROS에서 /tf 좌표가 들어올 때마다 실행
    private void OnTfReceived(Tf2.TFMessage msg)
    {
        if (msg == null || msg.transforms == null) return;

        foreach (var t in msg.transforms)
        {
            string parent = t.header.frame_id.TrimStart('/');
            string child = t.child_frame_id.TrimStart('/');

            var q = t.transform.rotation;
            float yaw = (float)Math.Atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            float x = (float)t.transform.translation.x;
            float y = (float)t.transform.translation.y;

            lock (lockObj)
            {
                if (parent == mapFrame && child == odomFrame)
                {
                    moX = x; moY = y; moYaw = yaw;
                    hasMapToOdom = true;
                }
                else if (parent == odomFrame && child == baseFrame)
                {
                    obX = x; obY = y; obYaw = yaw;
                    hasOdomToBase = true;
                }
            }
        }
    }

    void Update()
    {
        // M키를 누르면 언제든 다시 위치 탐색 실행 가능
        if (Input.GetKeyDown(KeyCode.M))
        {
            RequestGlobalLocalization();
        }

        UpdateUnityRobotTransform();
    }

    // 실제 로봇 좌표를 Unity MapOrigin 기준 좌표로 변환해 적용
    private void UpdateUnityRobotTransform()
    {
        float rx, ry, ryaw;

        lock (lockObj)
        {
            if (!hasMapToOdom || !hasOdomToBase) return;

            float c = Mathf.Cos(moYaw);
            float s = Mathf.Sin(moYaw);
            rx = moX + c * obX - s * obY;
            ry = moY + s * obX + c * obY;
            ryaw = moYaw + obYaw;
        }

        // ROS 좌표계(X앞, Y왼쪽) -> Unity 좌표계(Z앞, -X왼쪽) 변환
        Vector3 localPos = new Vector3(-ry, floorHeight, rx);
        unityRobot.position = mapOrigin.TransformPoint(localPos);
        unityRobot.rotation = mapOrigin.rotation * Quaternion.Euler(0f, -ryaw * Mathf.Rad2Deg + yawOffset, 0f);

        if (!isSynced)
        {
            isSynced = true;
            Debug.Log($"[RobotPoseSync] 위치 동기화 시작! 실제 로봇 좌표: X={rx:F2}m, Y={ry:F2}m");
        }
    }
}