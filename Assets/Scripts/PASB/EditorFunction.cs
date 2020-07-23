using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

public class EditorFunction : MonoBehaviour
{
    public string add_pose_name = "Idle";
    public bool calculate_feet_data = true;
    /// <summary>
    /// Calculate the pole direction by relation of leg joints positions
    /// </summary>
    public bool calculate_pole_by_leg = true;
    /// <summary>
    /// Use the direction from feet to toes as pole direction
    /// </summary>
    public bool calculate_pole_by_feet = false;
    /// <summary>
    /// Manually provide the word pole position;
    /// </summary>
    public Vector3 left_leg_pole_world_position;
    public Vector3 right_leg_pole_world_position;
    /// <summary>
    /// Record the pose as balance pose
    /// </summary>
    public void RecordBalancePose()
    {
        var pab = GameObject.FindObjectOfType<PhysicalAnimationBalance>();
        if (pab.root_rg == null)
            pab.root_rg = GetComponentInChildren<Rigidbody>();
        if (pab.all_rgs.Length == 0)
            pab.all_rgs = pab.GetComponentsInChildren<Rigidbody>();

        pab.CalculateCenterOfMass();

        Quaternion rotation_bias = Quaternion.Inverse(pab.center_of_all_mass.rotation) * pab.CoM_rotator.rotation;
        pab.CoM_balance_rotation_bias.Add(rotation_bias);

        if (pab.legs.Length == 0)
        {
            if (pab.feets.Length == 0)
                pab.feets = pab.GetComponentsInChildren<BalanceFeet>();
            pab.legs = new PhysicalLeg[pab.feets.Length];
            for (int i = 0; i < pab.feets.Length; i++)
                pab.legs[i] = new PhysicalLeg(pab.feets[i]);
        }
        for (int i = 0; i < pab.feets.Length; i++)
        {
            // Add new legs parameters
            pab.legs[i].ResetParameters(pab.center_of_all_mass, pab.feets[i], add_pose_name, calculate_feet_data, 
                calculate_pole_by_leg, calculate_pole_by_feet, pab.feets[i].is_right ? right_leg_pole_world_position : left_leg_pole_world_position);            
        }
    }
}
[CustomEditor(typeof(EditorFunction))]
public class EditorFunctionEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        EditorFunction editorFunction = (EditorFunction)target;
        if(GUILayout.Button("Record balance pose"))
        {
            editorFunction.RecordBalancePose();
        }
    }
}
