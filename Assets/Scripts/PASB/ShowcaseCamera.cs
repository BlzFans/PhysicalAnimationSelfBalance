using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ShowcaseCamera : MonoBehaviour
{
    [Space(5f)][Header("Camera parameters")]
    public Transform center_transform; 
    public float rotate_speed = 10f;
    public float zoom_speed = 5f;
    public float angle_limit = 90f;
    public float max_distance_limit = 10f;
    public float min_distance_limit = 1f;

    Vector3 center_last_position;
    // Start is called before the first frame update
    void Start()
    {
        center_last_position = center_transform.position;
    }

    // Update is called once per frame
    void Update()
    {
        // Position follow the center
        transform.position += center_transform.position - center_last_position;
        center_last_position = center_transform.position;

        // Rotate around center at horizontal
        float move_up = -Input.GetAxis("Mouse Y");
        // Limit the angle of camera
        float angle_diff = Vector3.SignedAngle(Vector3.up, transform.up, transform.right);
        float angle_sign = angle_diff < 0f ? -1f : 1f;
        float rotate_angle = rotate_speed * move_up;
        rotate_angle = Mathf.Abs(angle_diff + rotate_angle) < angle_limit ? rotate_angle : angle_sign * angle_limit - angle_diff;
        transform.RotateAround(center_transform.position, transform.right, rotate_angle);
        // Rotate around center at vertical
        float move_right = Input.GetAxis("Mouse X");
        transform.RotateAround(center_transform.position, Vector3.up, rotate_speed * move_right);

        // Zoom in or out center with limit
        float zoom_in = Input.GetAxis("Mouse ScrollWheel");
        Vector3 to_center_direction = center_transform.position - transform.position;
        float zoom_length = zoom_in * zoom_speed;
        float next_radius = to_center_direction.magnitude - zoom_length;
        if (next_radius > max_distance_limit)
            zoom_length = to_center_direction.magnitude - max_distance_limit;
        else if (next_radius < min_distance_limit)
            zoom_length = to_center_direction.magnitude - min_distance_limit;
        transform.position += to_center_direction.normalized * zoom_length;
    }
}
