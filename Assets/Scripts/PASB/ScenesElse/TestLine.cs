using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestLine : MonoBehaviour
{
    Vector3 project_direction = Vector3.left;
    public float min_keep_distance = 0.25f;
    public Transform point_knee;
    public Transform point_feet;
    public Transform other_point_knee;
    public Transform other_point_feet;
    

    // Update is called once per frame
    void Update()
    {
        Vector3 project_knee_point = Vector3.ProjectOnPlane(point_knee.position, project_direction);
        Vector3 project_feet_point = Vector3.ProjectOnPlane(point_feet.position, project_direction);
        Vector3 project_other_knee_point = Vector3.ProjectOnPlane(other_point_knee.position, project_direction);
        Vector3 project_other_feet_point = Vector3.ProjectOnPlane(other_point_feet.position, project_direction);

        Vector3 knee_closest_point_dir = project_knee_point - GetClosestPointOnFiniteLine(project_knee_point, project_other_feet_point, project_other_knee_point);
        Vector3 feet_closest_point_dir = project_feet_point - GetClosestPointOnFiniteLine(project_feet_point, project_other_feet_point, project_other_knee_point);
        Vector3 other_knee_closest_point_dir = GetClosestPointOnFiniteLine(project_other_knee_point, project_feet_point, project_knee_point) - project_other_knee_point;
        Vector3 other_feet_closest_point_dir = GetClosestPointOnFiniteLine(project_other_feet_point, project_feet_point, project_knee_point) - project_other_feet_point;

        Vector3 vertical_dir = Quaternion.AngleAxis(90f, Vector3.up) * project_direction.normalized;
        bool leg_overlap = (Vector3.Dot(vertical_dir, knee_closest_point_dir) < 0f || knee_closest_point_dir.magnitude < min_keep_distance) || 
                            (Vector3.Dot(vertical_dir, feet_closest_point_dir) < 0f || feet_closest_point_dir.magnitude < min_keep_distance) || 
                            (Vector3.Dot(vertical_dir, other_knee_closest_point_dir) < 0f || other_knee_closest_point_dir.magnitude < min_keep_distance) || 
                            (Vector3.Dot(vertical_dir, other_feet_closest_point_dir) < 0f || other_feet_closest_point_dir.magnitude < min_keep_distance);

        Debug.Log(leg_overlap);
        Debug.DrawLine(project_knee_point, project_feet_point, Color.red);
        Debug.DrawLine(project_other_knee_point, project_other_feet_point, Color.red);
        Debug.DrawLine(project_knee_point, project_knee_point - knee_closest_point_dir, Color.yellow);
        Debug.DrawLine(project_feet_point, project_feet_point - feet_closest_point_dir, Color.yellow);
        Debug.DrawLine(project_other_knee_point, project_other_knee_point + other_knee_closest_point_dir, Color.yellow);
        Debug.DrawLine(project_other_feet_point, project_other_feet_point + other_feet_closest_point_dir, Color.yellow);
    }

    Vector3 GetClosestPointOnFiniteLine(Vector3 point, Vector3 line_start, Vector3 line_end)
    {
        Vector3 line_direction = line_end - line_start;
        float line_length = line_direction.magnitude;
        line_direction.Normalize();
        float project_length = Mathf.Clamp(Vector3.Dot(point - line_start, line_direction), 0f, line_length);
        return line_start + line_direction * project_length;
    }
}
