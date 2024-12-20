using System;
using System.Collections.Generic;
using System.Threading;
using Unity.VisualScripting;
using Unity.VisualScripting.Dependencies.Sqlite;
using UnityEditor.UIElements;
using UnityEngine;
using UnityEngine.InputSystem.Interactions;
using UnityEngine.Scripting;

public class MakePrediction : MonoBehaviour
{
    public Transform allPredictionsHolder;
    public Prediction longPred, shortPred;


    public Transform longPredictionLineHolder, shortPredictionLineHolder;

    List<Vector3> allPredictions = new List<Vector3>();

    public LayerMask obstacleLayer;

    public GameObject testObject;

    bool refresh = false;


    void Start()
    {

        shortPred = new Prediction(true, 30, 1, 0, shortPredictionLineHolder);
        //longPred = new Prediction(false, 15, 3, 1, longPredictionLineHolder);

        launchPreditionThread(shortPred);
    }

    void StartPrediction(Prediction pred)
    {
        Vector3 alignementVector = pred.alignementVector;   

        for(int i = 0; i < pred.deep; i++)
        {
            foreach (DroneFake drone in pred.dronesPrediction)
            {
                drone.startPrediction(pred.dronesPrediction,alignementVector);
            }


            for(int j = 0; j < pred.dronesPrediction.Count; j++)
            {
                pred.dronesPrediction[j].UpdatePositionPrediction(pred.step);
                pred.allData[j].positions.Add(pred.dronesPrediction[j].position);
                pred.allData[j].crashed.Add(pred.dronesPrediction[j].hasCrashed);

                if (pred.dronesPrediction[j].hasCrashed && !pred.allData[j].crashedPrediction)
                {
                    pred.allData[j].crashedPrediction = true;
                    pred.allData[j].idFirstCrash = i;
                }
            }
        }

        pred.donePrediction = true;
    }

    void spawnPredictions()
    {
       // spawnPrediction(longPred);
        spawnPrediction(shortPred);
    }

    void launchPreditionThread(Prediction pred)
    {
        pred.alignementVector = MigrationPointController.alignementVector;

   //     pred.lastMigrationPoint = this.GetComponent<MigrationPointController>().migrationPoint;


        //start a thread with short prediction
        shortPred.directionOfMigration = this.GetComponent<MigrationPointController>().deltaMigration;
        spawnPredictions();
        lock (shortPred)
        {
            new Thread(() => StartPrediction(shortPred)).Start();
        }
    }
    void spawnPrediction(Prediction pred)
    {
        pred.allData = new List<DroneDataPrediction>();
        pred.dronesPrediction = new List<DroneFake>();

        foreach (Transform child in swarmModel.swarmHolder.transform)
        {
            DroneDataPrediction data = new DroneDataPrediction();
            pred.dronesPrediction.Add(new DroneFake(child.transform.position, child.GetComponent<DroneController>().droneFake.velocity, false));
            pred.allData.Add(data);
        }
    }

    void Update()
    {   
        if(shortPred.donePrediction)
        {
            this.GetComponent<HapticsTest>().HapticsPrediction(shortPred);
            UpdateLines(shortPred);
            shortPred.donePrediction = false;
            launchPreditionThread(shortPred);
        }
    }

    //on exit
    void OnDisable()
    {
        StopAllCoroutines();
    }

    void UpdateLines(Prediction pred)
    {
        if (pred.allData == null || pred.allData.Count == 0)
        {
            return; // Exit if no data to draw

        }

        // Destroy all existing line renderers
        foreach (LineRenderer line in pred.LineRenderers)
        {
            Destroy(line.gameObject);
        }
        pred.LineRenderers.Clear();

        int downsampleRate = 1; // Select 1 point every 5 data points

        foreach (DroneDataPrediction data in pred.allData)
        {
            float fractionOfPath = (float)data.idFirstCrash / data.positions.Count;
            Color purpleColor = new Color(0.5f, 0f, 0.5f, 1f); // Purple
            Color greyColor = new Color(0.5f, 0.5f, 0.5f, 0.2f); // Grey
            Color colorPath = Color.Lerp(greyColor, purpleColor, fractionOfPath);

            for(int i = 0; i < data.positions.Count - 1; i++)
            {
                if (i % downsampleRate == 0)
                {

                    bool isCrashed = data.crashed[i];
                    Color segmentColor = isCrashed ? Color.red : colorPath;
                    LineRenderer line = new GameObject().AddComponent<LineRenderer>();
                    line.transform.SetParent(pred.lineHolder);
                    line.positionCount = 2;
                    line.SetPosition(0, data.positions[i]);
                    line.SetPosition(1, data.positions[i + 1]);
                    line.startWidth = 0.1f;
                    line.endWidth = 0.1f;
                    line.material = new Material(Shader.Find("Unlit/Color"));
                    line.material.color = segmentColor;

                    pred.LineRenderers.Add(line);                    
                } 
            }   
        }
    }
}



public class Prediction
{
    public bool donePrediction = false;
    public bool shortPrediction;
    public int deep;
    public int current;

    public Vector3 directionOfMigration;

    public int step = 1;

    public Transform lineHolder;    

    public List<DroneFake> dronesPrediction;

    public List<DroneDataPrediction> allData;
    public List<LineRenderer> LineRenderers;

    public Vector3 alignementVector;

    public Prediction(bool prediction, int deep, int step, int current,  Transform lineHolder)
    {
        this.shortPrediction = prediction;
        this.deep = deep;
        this.step = step;
        this.current = current;
        this.lineHolder = lineHolder;
        this.allData = new List<DroneDataPrediction>();
        this.LineRenderers = new List<LineRenderer>();
        directionOfMigration = Vector3.zero;
    }

}

public class DroneDataPrediction
{
    public List<Vector3> positions;
    public List<bool> crashed;
    public bool crashedPrediction = false;
    public int idFirstCrash = 0;

    public DroneDataPrediction()
    {
        positions = new List<Vector3>();
        crashed = new List<bool>();
    }
}
