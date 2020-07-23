using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LimbIK: MonoBehaviour
{
	public Transform upperArm;
	public Transform forearm;
	public Transform hand;
	public Transform elbow;
	public Transform target;
    public float max_forearm_angle = 180f;
    public float min_forearm_angle = 0f;
	float angle;
	float upperArm_Length;
	float forearm_Length;
	float max_arm_Length;
    float min_arm_Length;
	float target_Length;

    List<Tuple<Vector3, Vector3>> draw_lines = new List<Tuple<Vector3, Vector3>>();
    List<Color> lines_colors = new List<Color>();
    List<Vector3> draw_triangles = new List<Vector3>();
    void Start() 
    {
        upperArm_Length = Vector3.Distance (upperArm.position, forearm.position);
        forearm_Length =  Vector3.Distance (forearm.position, hand.position);

        max_arm_Length = Mathf.Sqrt(Mathf.Pow(upperArm_Length, 2f) + Mathf.Pow(forearm_Length, 2f) - 2f * upperArm_Length * forearm_Length * Mathf.Cos(max_forearm_angle / Mathf.Rad2Deg));
        min_arm_Length = Mathf.Sqrt(Mathf.Pow(upperArm_Length, 2f) + Mathf.Pow(forearm_Length, 2f) - 2f * upperArm_Length * forearm_Length * Mathf.Cos(min_forearm_angle / Mathf.Rad2Deg));
    }
    void FixedUpdate() 
    {
        // Debug
        draw_lines = new List<Tuple<Vector3, Vector3>>();
        lines_colors = new List<Color>();
        draw_triangles = new List<Vector3>();

        Vector3 ik_plane_normal = Vector3.Cross(target.position - upperArm.position, elbow.position - upperArm.position);
        Vector3 target_c = target.position - upperArm.position;
        target_Length = Mathf.Max(Mathf.Min(target_c.magnitude, max_arm_Length), min_arm_Length);
        target_c = target_c.normalized * target_Length;

        angle = LawOfCosines(upperArm_Length, target_Length, forearm_Length);
        Vector3 target_a = Quaternion.AngleAxis(angle, ik_plane_normal) * target_c.normalized * upperArm_Length;

        draw_triangles.Add(upperArm.position);
        draw_triangles.Add(upperArm.position + target_a);
        draw_triangles.Add(upperArm.position + target_c);

        Vector3 ik_plane_center = (elbow.position + upperArm.position + target_a + upperArm.position + target_c) / 3f;
        draw_lines.Add(new Tuple<Vector3, Vector3>(ik_plane_center, ik_plane_center + ik_plane_normal));
        lines_colors.Add(Color.blue);
        Vector3 through = (upperArm.position + target_a - elbow.position).normalized * 2f;
        draw_lines.Add(new Tuple<Vector3, Vector3>(elbow.position, elbow.position + through));
        lines_colors.Add(Color.black);
    }
    float LawOfCosines(float a, float b, float c)
    {
        /*
               c
           B ______ A
             \    /
            a \  / b
               \/
               C
        */
        float cosC = (Mathf.Pow(a, 2f) + Mathf.Pow(b, 2f) - Mathf.Pow(c, 2f)) / (2f * a * b);
        float C = Mathf.Acos(Mathf.Clamp(cosC, -1f, 1f)) * Mathf.Rad2Deg;
        return C;
    }

    void OnDrawGizmos()
    {
        // Draw IK triangles
        for (int i = 0; i < draw_triangles.Count; i++)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawLine(draw_triangles[i], draw_triangles[(i+1) % 3 == 0 ? i-2 : i+1]);
            Gizmos.DrawSphere(draw_triangles[i], 0.01f);
        }
        
        for (int i = 0; i < draw_lines.Count; i++)
        {
            Debug.DrawLine(draw_lines[i].Item1, draw_lines[i].Item2, lines_colors[i]);
        }
    }
}

