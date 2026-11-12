using System;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;
using RosSharp.RosBridgeClient;

// =====================================================================================
//  ArmServoPublisher — LanderPiTeleop의 팔/그리퍼 움직임 → 실제 로봇팔 서보 전송
//  붙이는 곳: RosConnector와 LanderPiTeleop이 붙어 있는 오브젝트 (rosmentor)
// =====================================================================================

[Serializable]
public class BusServoPosition : Message
{
    public const string RosMessageName = "ros_robot_controller_msgs/msg/BusServoPosition";
    public ushort id;
    public ushort position;

    public BusServoPosition(ushort id, ushort position)
    {
        this.id = id;
        this.position = position;
    }
}

[Serializable]
public class SetBusServoPosition : Message
{
    public const string RosMessageName = "ros_robot_controller_msgs/msg/SetBusServoPosition";
    public double duration;
    public BusServoPosition[] position;

    public SetBusServoPosition()
    {
        duration = 0.15;
        position = new BusServoPosition[0];
    }
}

public class ArmServoPublisher : UnityPublisher<SetBusServoPosition>
{
    [Header("연동 대상 (비워두면 같은 오브젝트에서 자동 탐색)")]
    public LanderPiTeleop teleop;

    [Header("전송 주기 및 반응 속도")]
    public float publishRate = 10f;     // 초당 전송 횟수 (10Hz)
    public float servoDuration = 0.15f; // 서보 목표 도달 시간 (초)

    [Header("서보 방향 보정 (유니티와 실제 팔이 반대로 움직이면 -1로 변경)")]
    public int[] directionSign = { 1, 1, 1, 1, 1 };

    [Header("서보 중앙 기준값 (기본 500, ±120° = 0~1000)")]
    public int[] centerPulse = { 500, 500, 500, 500, 500 };

    [Header("그리퍼(10번 서보) 열림/닫힘 펄스 값")]
    public ushort gripperOpenPulse = 250;
    public ushort gripperClosePulse = 650;

    [Header("디버그")]
    public bool debugLog = false;

    private RosConnector connector;
    private bool ready = false;
    private float timer = 0f;

    // LanderPiTeleop 내부 private 필드(arm, gripperOpen) 읽기용
    private FieldInfo armField;
    private FieldInfo gripperField;
    private FieldInfo valueField;

    private ushort[] lastSent = new ushort[6];
    private bool hasSentOnce = false;

    protected override void Start()
    {
        if (string.IsNullOrEmpty(Topic))
            Topic = "/ros_robot_controller/bus_servo/set_position";

        if (teleop == null)
            teleop = GetComponent<LanderPiTeleop>();

        CacheReflectionFields();
        StartCoroutine(AdvertiseWhenConnected());
    }

    void CacheReflectionFields()
    {
        if (teleop == null) return;
        var flags = BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Public;
        armField = typeof(LanderPiTeleop).GetField("arm", flags);
        gripperField = typeof(LanderPiTeleop).GetField("gripperOpen", flags);

        var jointType = typeof(LanderPiTeleop).GetNestedType("Joint", BindingFlags.NonPublic);
        if (jointType != null)
            valueField = jointType.GetField("value", BindingFlags.Public | BindingFlags.Instance);
    }

    IEnumerator AdvertiseWhenConnected()
    {
        connector = GetComponent<RosConnector>();
        float waited = 0f;

        while (connector == null || connector.RosSocket == null || connector.IsConnected == null || !connector.IsConnected.WaitOne(0))
        {
            waited += Time.unscaledDeltaTime;
            if (connector != null && waited > connector.SecondsTimeout + 1f)
            {
                Debug.LogError("[ArmServoPub] RosBridge 연결 실패");
                yield break;
            }
            yield return null;
        }

        base.Start();
        ready = true;
        Debug.Log($"[ArmServoPub] 연결 완료, {Topic} 등록됨");
    }

    void Update()
    {
        if (!ready || teleop == null || armField == null || valueField == null) return;

        timer += Time.deltaTime;
        if (timer < 1f / publishRate) return;
        timer = 0f;

        var armList = armField.GetValue(teleop) as IList;
        if (armList == null || armList.Count < 5) return;

        ushort[] current = new ushort[6];
        bool changed = !hasSentOnce;

        // 1~5번 관절 각도(-120° ~ +120°) -> 서보 펄스(0 ~ 1000) 변환
        for (int i = 0; i < 5; i++)
        {
            float deg = (float)valueField.GetValue(armList[i]);
            int sign = (directionSign != null && i < directionSign.Length) ? directionSign[i] : 1;
            int center = (centerPulse != null && i < centerPulse.Length) ? centerPulse[i] : 500;

            // 240도 범위 = 1000 펄스 -> 1도당 약 4.1667 펄스
            int pulse = Mathf.RoundToInt(center + (deg * sign * (1000f / 240f)));
            current[i] = (ushort)Mathf.Clamp(pulse, 0, 1000);

            if (current[i] != lastSent[i]) changed = true;
        }

        // 10번 그리퍼 상태 변환
        bool isOpen = gripperField != null && (bool)gripperField.GetValue(teleop);
        current[5] = isOpen ? gripperOpenPulse : gripperClosePulse;
        if (current[5] != lastSent[5]) changed = true;

        // 값이 바뀌었을 때만 토픽 전송 (통신 부하 방지)
        if (changed)
        {
            SendServoMessage(current);
            Array.Copy(current, lastSent, 6);
            hasSentOnce = true;
        }
    }

    void SendServoMessage(ushort[] p)
    {
        SetBusServoPosition msg = new SetBusServoPosition
        {
            duration = servoDuration,
            position = new BusServoPosition[]
            {
                new BusServoPosition(1, p[0]),
                new BusServoPosition(2, p[1]),
                new BusServoPosition(3, p[2]),
                new BusServoPosition(4, p[3]),
                new BusServoPosition(5, p[4]),
                new BusServoPosition(10, p[5])
            }
        };

        Publish(msg);

        if (debugLog)
            Debug.Log($"[ArmServoPub] 전송: J1={p[0]}, J2={p[1]}, J3={p[2]}, J4={p[3]}, J5={p[4]}, Grip={p[5]}");
    }
}