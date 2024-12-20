using System;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class DroneController : MonoBehaviour
{

#region Parameters
    private swarmModel swarm;

    public Vector3 separationForce = Vector3.zero;
    public Vector3 alignmentForce = Vector3.zero;
    public Vector3 cohesionForce = Vector3.zero;
    public Vector3 migrationForce = Vector3.zero;
    public Vector3 obstacleAvoidanceForce = Vector3.zero;

    public Material connectedColor;
    public Material farColor;
    public Material notConnectedColor;
    public Material embodiedColor;

    public bool showGuizmos = false;
    public bool prediction = false;


    private GameObject gm;
    private float timeSeparated = 0;
    public GameObject fireworkParticle;

    public DroneFake droneFake;

    float realScore
    {
        get
        {
            return HapticAudioManager.GetDroneNetworkScore(this.gameObject);
        }
    }
    
    #endregion
    
    
    void Start()
    {
        StartNormal();
        Application.targetFrameRate = 30; // Set the target frame rate to 30 FPS
    }

    public void crash()
    {
        if (CameraMovement.embodiedDrone == this.gameObject)
        {
            CameraMovement.desembodiedDrone();
            CameraMovement.nextEmbodiedDrone = null;
        }

        gm.GetComponent<swarmModel>().RemoveDrone(this.gameObject);

        GameObject firework = Instantiate(fireworkParticle, transform.position, Quaternion.identity);
        firework.transform.position = transform.position;
        Destroy(firework, 0.5f);
    }

    void Update()
    {
        UpdateNormal();
    }


    #region NormalMode
    void StartNormal()
    {
        gm = GameObject.FindGameObjectWithTag("GameManager");

        swarm = gm.GetComponent<swarmModel>();

//        swarm.GetComponent<sendInfoGameObject>().setupCallback(() =>
//        {
//            return new DataEntry(this.transform.name + "_position", transform.position.ToString());
//       });

//        swarm.GetComponent<sendInfoGameObject>().setupCallback(() =>
//        {
//            return new DataEntry(this.transform.name + "_velocity", velocity.ToString());
//        });
    }

    void UpdateNormal()
    {
        transform.position = droneFake.position;
        updateColor();
        updateSound();
    }

    #endregion


    #region HapticAudio
    void updateColor()
    {
        if (CameraMovement.embodiedDrone == this.gameObject || CameraMovement.nextEmbodiedDrone == this.gameObject || MigrationPointController.selectedDrone == this.gameObject)
        {
            this.GetComponent<Renderer>().material = embodiedColor;
            return;
        }

        float score = realScore;

        if (score < -0.9f)
        {
            this.GetComponent<Renderer>().material = notConnectedColor;
        }
        else if (score < 1)
        {
            this.GetComponent<Renderer>().material.Lerp(farColor, connectedColor, score);
        }
        else // == 1
        {
            this.GetComponent<Renderer>().material = connectedColor;
        }
    }

    void updateSound()
    {
        float score = realScore;

        if (score < -0.9f)
        {
            timeSeparated += Time.deltaTime;
            this.GetComponent<AudioSource>().enabled = HapticAudioManager.GetAudioSourceCharacteristics(timeSeparated);
        }
        else
        {
            timeSeparated = 0;
            this.GetComponent<AudioSource>().enabled = false;
        }

    }

    #endregion

}


public class ObstacleInRange
{
    public Vector3 position;
    public float distance;

    public ObstacleInRange(Vector3 position, float distance)
    {
        this.position = position;
        this.distance = distance;
    }
}