using System.Collections.Generic;
using UnityEngine;
using RosSharp;                       // Unity2Ros 확장
using RosSharp.RosBridgeClient;
using Tf2 = RosSharp.RosBridgeClient.MessageTypes.Tf2;
using Geometry = RosSharp.RosBridgeClient.MessageTypes.Geometry;
using Std = RosSharp.RosBridgeClient.MessageTypes.Std;

/// <summary>
/// LanderPi(rosmentor)용 /tf 발행기. 팀의 TfPublisher(터틀봇 전용 링크 이름)를 대신합니다.
///   odom → base_footprint        (로봇의 현재 위치, 시작점 = odom 원점)
///   base_footprint → base_link    (고정)
///   base_link → lidar_frame       (고정, LiDAR 위치)
/// 링크 사이 위치는 화면에 보이는 실제 위치에서 계산하므로, 중간 링크 구조나 크기(Scale)가 달라도 맞습니다.
/// 붙이는 곳: rosmentor (RosConnector와 같은 오브젝트 또는 그 아래)
/// </summary>
public class LanderPiSimTf : MonoBehaviour
{
    public string Topic = "tf";
    public string OdomFrame = "odom";
    public string LidarFrame = "lidar_frame";
    public float PublishRateHz = 30f;

    RosConnector connector;
    string pubId;
    bool advertised;
    float next;
    Transform footprint, baseLink, lidar;

    void Start()
    {
        connector = GetComponentInParent<RosConnector>();
        footprint = FindLink("base_footprint");
        baseLink = FindLink("base_link");
        lidar = FindLink(LidarFrame);
        if (footprint == null) Debug.LogError("[LanderPiSimTf] base_footprint를 못 찾았어요.");
        if (lidar == null) Debug.LogWarning("[LanderPiSimTf] " + LidarFrame + "를 못 찾았어요. LiDAR 위치가 TF에 안 들어갑니다.");
    }

    // 메시 오브젝트(Visuals/unnamed/xxx)와 이름이 같을 수 있어서, 'Visuals' 자식이 있는 진짜 링크를 우선
    Transform FindLink(string n)
    {
        Transform fallback = null;
        foreach (var t in GetComponentsInChildren<Transform>(true))
        {
            if (t.name != n) continue;
            if (t.Find("Visuals") != null) return t;
            if (fallback == null) fallback = t;
        }
        return fallback;
    }

    void Update()
    {
        if (connector == null || connector.RosSocket == null || footprint == null) return;
        if (!advertised) { pubId = connector.RosSocket.Advertise<Tf2.TFMessage>(Topic); advertised = true; }
        if (Time.time < next) return;
        next = Time.time + 1f / PublishRateHz;

        var list = new List<Geometry.TransformStamped>();
        list.Add(Make(OdomFrame, "base_footprint", footprint.position, footprint.rotation));
        if (baseLink != null) list.Add(Relative("base_footprint", footprint, "base_link", baseLink));
        if (lidar != null) list.Add(Relative(baseLink != null ? "base_link" : "base_footprint",
                                             baseLink != null ? baseLink : footprint, LidarFrame, lidar));
        connector.RosSocket.Publish(pubId, new Tf2.TFMessage { transforms = list.ToArray() });
    }

    Geometry.TransformStamped Relative(string parentName, Transform parent, string childName, Transform child)
    {
        Quaternion inv = Quaternion.Inverse(parent.rotation);
        Vector3 pos = inv * (child.position - parent.position);   // 실제 보이는 거리(m)
        Quaternion rot = inv * child.rotation;
        return Make(parentName, childName, pos, rot);
    }

    Geometry.TransformStamped Make(string parent, string child, Vector3 pos, Quaternion rot)
    {
        Vector3 p = pos.Unity2Ros();
        Quaternion q = rot.Unity2Ros();
        var ts = new Geometry.TransformStamped
        {
            header = new Std.Header { frame_id = parent },
            child_frame_id = child,
            transform = new Geometry.Transform
            {
                translation = new Geometry.Vector3 { x = p.x, y = p.y, z = p.z },
                rotation = new Geometry.Quaternion { x = q.x, y = q.y, z = q.z, w = q.w }
            }
        };
        SimTime.Stamp(ts.header);
        return ts;
    }
}
