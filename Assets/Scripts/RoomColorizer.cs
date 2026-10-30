using System;
using System.Collections.Generic;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

/// <summary>
/// Revit에서 가져와 검정색으로 보이는 방에 "이름을 보고" 자동으로 색을 입힙니다.
/// (벽=밝은 베이지, 바닥=회갈색, 가구=나무색, 유리=반투명 하늘색, 덕트=은색 ...)
///
/// 붙이는 곳: 방 맨 위 오브젝트 (예: 프로젝트1)
/// 사용: Inspector에서 [🎨 색 입히기] 버튼 → Ctrl+S
///  - 색은 아래 Rules 목록에서 직접 바꿀 수 있어요 (바꾼 뒤 버튼 다시 누르기)
///  - 되돌리기: Ctrl+Z
///  - 만든 재질은 Assets/RoomMaterials 폴더에 저장됩니다
/// </summary>
public class RoomColorizer : MonoBehaviour
{
    [Serializable]
    public class Rule
    {
        public string name;
        [Tooltip("쉼표로 구분. 오브젝트 이름·부모 이름·원래 재질 이름에 하나라도 들어있으면 이 색")]
        public string keywords;
        public Color color = Color.white;
        [Range(0, 1)] public float smoothness = 0.2f;
        [Range(0, 1)] public float metallic = 0f;
        public bool transparent;
    }

    [Tooltip("위에서부터 먼저 맞는 규칙이 적용돼요")]
    public List<Rule> rules = new List<Rule>
    {
        R("유리",   "glass,glazing,window,curtain wall,유리,창호,창문",           new Color(0.65f, 0.82f, 0.95f, 0.35f), 0.9f, 0f, true),
        R("천장",   "ceiling,천장,천정",                                            new Color(0.95f, 0.95f, 0.94f)),
        R("바닥",   "floor,slab,바닥,슬래브",                                       new Color(0.60f, 0.57f, 0.52f), 0.3f),
        R("문",     "door,문틀,도어,문",                                            new Color(0.55f, 0.38f, 0.24f), 0.3f),
        R("가구",   "furniture,furni,desk,table,chair,cabinet,shelf,casework,가구,책상,의자,테이블,수납,선반,캐비닛", new Color(0.72f, 0.56f, 0.38f), 0.3f),
        R("공조기", "hvac,ahu,fcu,aircon,air conditioner,에어컨,공조,냉난방,실내기,실외기", new Color(0.93f, 0.94f, 0.96f), 0.4f),
        R("덕트",   "duct,diffuser,grille,덕트,디퓨저,그릴",                        new Color(0.74f, 0.76f, 0.79f), 0.6f, 0.7f),
        R("배관",   "pipe,piping,valve,배관,밸브,파이프",                           new Color(0.20f, 0.45f, 0.78f), 0.5f, 0.3f),
        R("전기",   "elect,electrical,lighting,light,switch,outlet,panel,전기,조명,스위치,콘센트,분전", new Color(0.96f, 0.95f, 0.88f), 0.3f),
        R("벽",     "wall,partition,column,벽,기둥,칸막이",                         new Color(0.88f, 0.86f, 0.82f)),
        R("기본",   "",                                                              new Color(0.80f, 0.80f, 0.80f)),
    };

    static Rule R(string n, string k, Color c, float s = 0.2f, float m = 0f, bool t = false) =>
        new Rule { name = n, keywords = k, color = c, smoothness = s, metallic = m, transparent = t };

    [Tooltip("켜두면 Play할 때마다 다시 입힘 (보통은 꺼두고 버튼으로 한 번만)")]
    public bool applyOnPlay = false;

    void Start() { if (applyOnPlay) Apply(); }

    [ContextMenu("🎨 색 입히기")]
    public void Apply()
    {
        var mats = new Dictionary<Rule, Material>();
        var count = new Dictionary<string, int>();
        foreach (var r in GetComponentsInChildren<Renderer>(true))
        {
            if (r is ParticleSystemRenderer || r is LineRenderer || r is TrailRenderer) continue;
            string chain = NameChain(r.transform);
            var old = r.sharedMaterials;
            var neu = new Material[Mathf.Max(1, old.Length)];
            for (int i = 0; i < neu.Length; i++)
            {
                string matName = i < old.Length && old[i] != null ? old[i].name.ToLowerInvariant() : "";
                Rule rule = Match(matName) ?? Match(chain) ?? rules[rules.Count - 1];
                if (!mats.TryGetValue(rule, out var m)) mats[rule] = m = MakeMaterial(rule);
                neu[i] = m;
                count[rule.name] = count.TryGetValue(rule.name, out var c) ? c + 1 : 1;
            }
#if UNITY_EDITOR
            if (!Application.isPlaying) Undo.RecordObject(r, "Room colors");
#endif
            r.sharedMaterials = neu;
        }
#if UNITY_EDITOR
        if (!Application.isPlaying)
        {
            AssetDatabase.SaveAssets();
            UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(gameObject.scene);
        }
#endif
        var parts = new List<string>();
        foreach (var kv in count) parts.Add($"{kv.Key} {kv.Value}");
        Debug.Log("[RoomColorizer] 색 입힘: " + string.Join(", ", parts));
    }

    Rule Match(string text)
    {
        if (string.IsNullOrEmpty(text)) return null;
        foreach (var rule in rules)
        {
            if (string.IsNullOrWhiteSpace(rule.keywords)) continue;
            foreach (var k in rule.keywords.Split(','))
            {
                string key = k.Trim().ToLowerInvariant();
                if (key.Length > 0 && text.Contains(key)) return rule;
            }
        }
        return null;
    }

    // 자기 이름 + 부모들 이름 (방 루트는 제외) — 메시 이름이 'unnamed' 여도 부모 이름으로 판단
    string NameChain(Transform t)
    {
        var s = new System.Text.StringBuilder();
        for (; t != null && t != transform; t = t.parent) s.Append(t.name).Append(' ');
        return s.ToString().ToLowerInvariant();
    }

    Material MakeMaterial(Rule rule)
    {
        Shader sh = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
        Material m = null;
#if UNITY_EDITOR
        string path = null;
        if (!Application.isPlaying)
        {
            if (!AssetDatabase.IsValidFolder("Assets/RoomMaterials")) AssetDatabase.CreateFolder("Assets", "RoomMaterials");
            path = $"Assets/RoomMaterials/Room_{rule.name}.mat";
            m = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (m != null) m.shader = sh;
        }
#endif
        if (m == null) m = new Material(sh) { name = "Room_" + rule.name };

        m.SetColor("_BaseColor", rule.color);
        m.SetColor("_Color", rule.color);
        m.SetFloat("_Smoothness", rule.smoothness);
        m.SetFloat("_Glossiness", rule.smoothness);
        m.SetFloat("_Metallic", rule.metallic);
        SetTransparent(m, rule.transparent);

#if UNITY_EDITOR
        if (path != null)
        {
            if (!AssetDatabase.Contains(m)) AssetDatabase.CreateAsset(m, path);
            else EditorUtility.SetDirty(m);
        }
#endif
        return m;
    }

    static void SetTransparent(Material m, bool on)
    {
        m.SetFloat("_Surface", on ? 1f : 0f);      // URP: 0 불투명, 1 반투명
        m.SetFloat("_Blend", 0f);
        m.SetFloat("_SrcBlend", on ? (float)UnityEngine.Rendering.BlendMode.SrcAlpha : (float)UnityEngine.Rendering.BlendMode.One);
        m.SetFloat("_DstBlend", on ? (float)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha : (float)UnityEngine.Rendering.BlendMode.Zero);
        m.SetFloat("_ZWrite", on ? 0f : 1f);
        if (on) { m.EnableKeyword("_SURFACE_TYPE_TRANSPARENT"); m.renderQueue = (int)UnityEngine.Rendering.RenderQueue.Transparent; }
        else { m.DisableKeyword("_SURFACE_TYPE_TRANSPARENT"); m.renderQueue = -1; }
        m.SetOverrideTag("RenderType", on ? "Transparent" : "Opaque");
    }
}

#if UNITY_EDITOR
[CustomEditor(typeof(RoomColorizer))]
public class RoomColorizerEditor : Editor
{
    public override void OnInspectorGUI()
    {
        if (GUILayout.Button("🎨 색 입히기", GUILayout.Height(32)))
            ((RoomColorizer)target).Apply();
        EditorGUILayout.HelpBox("색을 바꾸려면 아래 Rules에서 Color를 바꾸고 버튼을 다시 누르세요. 되돌리기는 Ctrl+Z, 저장은 Ctrl+S.", MessageType.Info);
        DrawDefaultInspector();
    }
}
#endif
