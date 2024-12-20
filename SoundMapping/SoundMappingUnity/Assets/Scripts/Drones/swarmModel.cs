using System;
using System.Collections;
using System.Collections.Generic;
using FischlWorks_FogWar;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.InputSystem.Interactions;

public class swarmModel : MonoBehaviour
{
    public static GameObject swarmHolder;
    public GameObject dronePrefab;
    public int numDrones = 10;
    public float spawnRadius = 10f;
    public static float spawnHeight = 5f;

    public float lastObstacleAvoidance = -1f;


    public float maxSpeed = 5f;
    public float maxForce = 10f;

    public static int extraDistanceNeighboor = 4;
    public static float neighborRadius
    {
        get
        {
            return desiredSeparation + extraDistanceNeighboor;
        }
    }

    public static float desiredSeparation = 3f;
    public float alpha = 1.5f; // Separation weight
    public float beta = 1.0f;  // Alignment weight
    public float delta = 1.0f; // Migration weight

    public float avoidanceRadius = 2f;     // Radius for obstacle detection
    public float avoidanceForce = 10f;     // Strength of the avoidance force

    public float droneRadius = 1.0f;      // Radius of the drone
    public LayerMask obstacleLayer;        // Layer mask for obstacles

    public csFogWar fogWar;

    public const int PRIORITYWHENEMBODIED = 2;
    public float dampingFactor = 0.98f;


    public List<DroneFake> drones = new List<DroneFake>();

    void Awake()
    {
        swarmHolder = GameObject.FindGameObjectWithTag("Swarm");
        spawn();
    }

    void Start()
    {
        Application.targetFrameRate = 30; // Set the target frame rate to 30 FPS
        
        this.GetComponent<sendInfoGameObject>().setupCallback(getAverageCohesion);
        this.GetComponent<sendInfoGameObject>().setupCallback(getAverageAlignment);
        this.GetComponent<sendInfoGameObject>().setupCallback(getAverageSeparation);
        this.GetComponent<sendInfoGameObject>().setupCallback(getAverageMigration);
        this.GetComponent<sendInfoGameObject>().setupCallback(getAverageObstacleAvoidance);
        this.GetComponent<sendInfoGameObject>().setupCallback(getDeltaAverageObstacle);
    }

    void refreshParameters()
    {
        DroneFake.maxForce = maxForce;
        DroneFake.maxSpeed = maxSpeed;
        DroneFake.desiredSeparation = desiredSeparation;
        DroneFake.alpha = alpha;
        DroneFake.beta = beta;

        DroneFake.delta = delta;
        DroneFake.avoidanceRadius = avoidanceRadius;
        DroneFake.avoidanceForce = avoidanceForce;
        DroneFake.droneRadius = droneRadius;
        DroneFake.neighborRadius = neighborRadius;
        DroneFake.obstacleLayer = obstacleLayer;
        DroneFake.PRIORITYWHENEMBODIED = PRIORITYWHENEMBODIED;
        DroneFake.dampingFactor = dampingFactor;
        DroneFake.spawnHeight = spawnHeight;
    }

    void FixedUpdate()
    {
        refreshParameters();
        if (Input.GetKeyDown(KeyCode.R))
        {
            spawn();
            this.GetComponent<Timer>().Restart();
        }

        foreach (DroneFake drone in drones)
        {
            drone.ComputeForces(drones, MigrationPointController.alignementVector);
        }

        foreach (Transform drone in swarmHolder.transform)
        {
            drone.GetComponent<DroneController>().droneFake.UpdatePositionPrediction(1);
            if (drone.GetComponent<DroneController>().droneFake.hasCrashed)
            {
                drone.GetComponent<DroneController>().crash();
            }
        }
    }

    void spawn()
    {
        fogWar.ResetMapAndFogRevealers();

        GameObject[] dronesToDelete = GameObject.FindGameObjectsWithTag("Drone");
        //kill all drones
        foreach (GameObject drone in dronesToDelete)
        {
            Destroy(drone.gameObject);
        }

        drones.Clear();


        for (int i = 0; i < numDrones; i++)
        {
            //spawn on a circle
            Vector3 spawnPosition = new Vector3(spawnRadius * Mathf.Cos(i * 2 * Mathf.PI / numDrones), spawnHeight, spawnRadius * Mathf.Sin(i * 2 * Mathf.PI / numDrones));
            
            GameObject drone = Instantiate(dronePrefab, spawnPosition, Quaternion.identity);

            drone.GetComponent<DroneController>().droneFake = new DroneFake(spawnPosition, Vector3.zero, false);

            fogWar.AddFogRevealer(drone.transform, 5, true);

            drones.Add(drone.GetComponent<DroneController>().droneFake);

            drone.transform.parent = swarmHolder.transform;
            drone.name = "Drone"+i.ToString();
        }

        this.GetComponent<HapticAudioManager>().Reset();
        this.GetComponent<DroneNetworkManager>().Reset();
    }

 
    public void RemoveDrone(GameObject drone)
    {
        if (drone.transform.parent == swarmHolder.transform)
        {
            drone.gameObject.SetActive(false);
            drone.transform.parent = null;
            //this.GetComponent<CameraMovement>().resetFogExplorers();
        }

        this.GetComponent<Timer>().DroneDiedCallback();

        if (swarmHolder.transform.childCount == 0)
        {
            this.GetComponent<Timer>().Restart();
            spawn();
        }

        drones.Remove(drone.GetComponent<DroneController>().droneFake);
    }

    DataEntry getAverageCohesion()
    {
        Vector3 averageCohesion = Vector3.zero;
        int numDrones = 0;

        foreach (Transform drone in swarmHolder.transform)
        {
            averageCohesion += drone.GetComponent<DroneController>().cohesionForce;
            numDrones++;
        }

        if (numDrones > 0)
        {
            averageCohesion /= numDrones;
        }

        return new DataEntry("averageCohesion", averageCohesion.magnitude.ToString(), fullHistory: true);
    }

    DataEntry getDeltaAverageObstacle()
    {
        Vector3 averageObstacle = Vector3.zero;
        int numDrones = 0;

        foreach (Transform drone in swarmHolder.transform)
        {
            averageObstacle += drone.GetComponent<DroneController>().obstacleAvoidanceForce;
            numDrones++;
        }

        if (numDrones > 0)
        {
            averageObstacle /= numDrones;
        }




        if(lastObstacleAvoidance < 0)
        {
            lastObstacleAvoidance = averageObstacle.magnitude;
            return new DataEntry("deltaObstacle", "0", fullHistory: true);
        }

        float delta = (averageObstacle.magnitude - lastObstacleAvoidance)*Time.deltaTime;
        lastObstacleAvoidance = averageObstacle.magnitude;


        return new DataEntry("deltaObstacle", delta.ToString(), fullHistory: true);
    }

    DataEntry getAverageAlignment()
    {
        Vector3 averageAlignment = Vector3.zero;
        int numDrones = 0;

        foreach (Transform drone in swarmHolder.transform)
        {
            averageAlignment += drone.GetComponent<DroneController>().alignmentForce;
            numDrones++;
        }

        if (numDrones > 0)
        {
            averageAlignment /= numDrones;
        }

        return new DataEntry("averageAlignment", averageAlignment.magnitude.ToString(), fullHistory: true);
    }

    DataEntry getAverageSeparation()
    {
        Vector3 averageSeparation = Vector3.zero;
        int numDrones = 0;

        foreach (Transform drone in swarmHolder.transform)
        {
            averageSeparation += drone.GetComponent<DroneController>().separationForce;
            numDrones++;
        }

        if (numDrones > 0)
        {
            averageSeparation /= numDrones;
        }

        return new DataEntry("averageSeparation", averageSeparation.magnitude.ToString(), fullHistory: true);
    }

    DataEntry getAverageMigration()
    {
        Vector3 averageMigration = Vector3.zero;
        int numDrones = 0;

        foreach (Transform drone in swarmHolder.transform)
        {
            averageMigration += drone.GetComponent<DroneController>().migrationForce;
            numDrones++;
        }

        if (numDrones > 0)
        {
            averageMigration /= numDrones;
        }

        return new DataEntry("averageMigration", averageMigration.magnitude.ToString(), fullHistory: true);
    }

    DataEntry getAverageObstacleAvoidance()
    {
        Vector3 averageObstacleAvoidance = Vector3.zero;
        int numDrones = 0;

        foreach (Transform drone in swarmHolder.transform)
        {
            averageObstacleAvoidance += drone.GetComponent<DroneController>().obstacleAvoidanceForce;
            numDrones++;
        }

        if (numDrones > 0)
        {
            averageObstacleAvoidance /= numDrones;
        }

        return new DataEntry("averageObstacleAvoidance", averageObstacleAvoidance.magnitude.ToString(), fullHistory: true);
    }

}


public class DroneFake
{
    #region Paramters Classes
    public Vector3 position;
    public Vector3 acceleration;
    public Vector3 velocity;
    
    public static float maxSpeed;
    public static float maxForce;
    public static float desiredSeparation = 3f;
    public static float neighborRadius = 10f;
    public static float alpha = 1.5f; // Separation weight
    public static float beta = 1.0f;  // Alignment weight
    public static float delta = 1.0f; // Migration weight
    public static float avoidanceRadius = 2f;     // Radius for obstacle detection
    public static float avoidanceForce = 10f;     // Strength of the avoidance force
    public static float droneRadius = 0.17f;

    public static float dampingFactor = 0.96f;

    public static float lastDT = 0.02f;

    public static float spawnHeight = 0.5f;

    public bool embodied = false;

    public static int PRIORITYWHENEMBODIED = 2;

    public bool hasCrashed = false;

    public static LayerMask obstacleLayer;

    #endregion

    public DroneFake(Vector3 position, Vector3 velocity, bool embodied)
    {
        this.position = position;
        this.velocity = velocity;
        this.embodied = embodied;
    }
    public List<DroneFake> GetNeighbors(List<DroneFake> allDrones)
    {
        List<DroneFake> neighbors = new List<DroneFake>();
        foreach (DroneFake drone in allDrones)
        {
            if (drone == this) continue;

            if (Vector3.Distance(this.position, drone.position) < neighborRadius)
            {
                if(drone.hasCrashed)
                {
                    continue;
                }
                neighbors.Add(drone);
            }
        }
        return neighbors;
    }

    public float GetCohesionIntensity(float r, float d_ref, float a, float b, float c)
    {
        float diff = r - d_ref;
        return (a + b) * 0.5f * (Mathf.Sqrt(1 + (diff + c) * (diff + c)) - Mathf.Sqrt(1 + c * c)) + (a - b) * diff * 0.5f;
    }

    // Calculate the cohesion intensity derivative for the Olfati-Saber model
    public float GetCohesionIntensityDer(float r, float d_ref, float a, float b, float c)
    {
        float diff = r - d_ref;
        return ((a + b) * 0.5f * (diff + c) / Mathf.Sqrt(1 + (diff + c) * (diff + c))) + (a - b) * 0.5f;
    }

    // Calculate the neighbour weight for the Olfati-Saber model
    public float GetNeighbourWeight(float r, float r0, float delta)
    {
        float r_ratio = r / r0;

        if (r_ratio < delta)
        {
            return 1.0f;
        }
        else if (r_ratio < 1.0f)
        {
            float arg = Mathf.PI * (r_ratio - delta) / (1.0f - delta);
            float val = 1.0f + Mathf.Cos(arg);
            return 0.25f * (val * val);
        }
        else
        {
            return 0.0f;
        }
    }

    // Calculate the derivative of the neighbour weight for the Olfati-Saber model
    public float GetNeighbourWeightDer(float r, float r0, float delta)
    {
        float r_ratio = r / r0;

        if (r_ratio < delta)
        {
            return 0.0f;
        }
        else if (r_ratio < 1.0f)
        {
            float arg = Mathf.PI * (r_ratio - delta) / (1.0f - delta);
            return 0.5f * (-Mathf.PI / (1.0f - delta)) * (1.0f + Mathf.Cos(arg)) * Mathf.Sin(arg);
        }
        else
        {
            return 0.0f;
        }
    }

    public float get_cohesion_force(float r, float d_ref, float a, float b, float c, float r0,float delta)
    {

        return 1 / r0 * GetNeighbourWeightDer(r, r0, delta) * GetCohesionIntensity(r, d_ref, a, b, c) + GetNeighbourWeight(r, r0, delta) * GetCohesionIntensityDer(r, d_ref, a, b, c);
    }

    public float get_neighbour_weight(float r, float r0, float delta)
    {

        float r_ratio = r / r0;

        if(r_ratio < delta)
        {
            return 1;
        }else if (r_ratio < 1)
        {
            return 0.25f * (1 + Mathf.Cos(Mathf.PI * (r_ratio - delta) / ((1 - delta)*(1 - delta))));
        }
        else
        {
            return 0;
        }

    }
    

    
    
    public void startPrediction(List<DroneFake> allDrones, Vector3 alignementVector)
    {
        ComputeForces(allDrones, alignementVector);
    }



    Vector3 computeAlignment(List<DroneFake> neighbors, Vector3 alignementVector)
    {

        return alignementVector;
    }

    public void ComputeForces(List<DroneFake> allDrones, Vector3 v_ref)
    {
        List<DroneFake> neighbors = GetNeighbors(allDrones);

        Vector3 alignmentForce = Vector3.zero;
        Vector3 cohesionForce = Vector3.zero;
        Vector3 obstacleAvoidanceForce = Vector3.zero;

        float d_ref = desiredSeparation;
        float d_ref_obs = 1.0f;

        float r0_coh = neighborRadius;            
        float delta = 1;

        float a = alpha;           
        float b = beta;   
        float c = (b - a)/(2*Mathf.Sqrt(a*b));

        float c_vm = 1;                             // Coefficient of velocity matching

        float r0_obs = avoidanceRadius;             // Radius of obstacle avoidance
        float lambda_obs = 1;                               // (0,1]
        float c_pm_obs = avoidanceForce;            // Coefficient of obstacle avoidance
        float c_vm_obs = 0f;                        // Coefficient of velocity matching


        // Compute the cohesion force
        foreach (DroneFake neighbor in neighbors)
        {
            int neighborPriority = CameraMovement.embodiedDrone == neighbor.embodied ? PRIORITYWHENEMBODIED : 1;

            Vector3 toNeighbor = neighbor.position - position;
            float distance = toNeighbor.magnitude - 2 * droneRadius;

            // Separation (repulsion)
            if (distance < 0)
            {
                hasCrashed = true;
            }

            cohesionForce += get_cohesion_force(distance, d_ref, a, b, c, r0_coh, delta)*toNeighbor.normalized;
        }

        // Compute the obstacle avoidance force
        List<Vector3> obstacles = ClosestPointCalculator.ClosestPointsWithinRadius(position, avoidanceRadius);

        foreach (Vector3 obstacle_pos in obstacles)
        {
            // Calculate a force away from the obstacle
            Vector3 awayFromObstacle = position - obstacle_pos;
            float dist = awayFromObstacle.magnitude - droneRadius;

            Vector3 pos_obs = obstacle_pos;
            Vector3 vel_obs = Vector3.zero;
            
            Vector3 pos_gamma = obstacle_pos + lambda_obs * v_ref.normalized;
            float d_ag = (pos_gamma - pos_obs).magnitude;

            obstacleAvoidanceForce += (
                c_pm_obs * get_neighbour_weight(dist, d_ref, delta) * (
                    get_cohesion_force(dist, d_ref_obs, a, b, c, r0_coh, delta) * (pos_obs - position) / dist +
                    get_cohesion_force(d_ag, d_ref_obs, a, b, c, r0_coh, delta) * (pos_gamma - position).normalized
                )
                + c_vm_obs * (vel_obs - velocity)
            );

            if (dist < 0)
            {
                hasCrashed = true;
            }
        }

        // Compute the alignment force
        alignmentForce = c_vm *(v_ref - velocity);



        if (embodied)
        {
            Vector3 force = MigrationPointController.alignementVector;
            acceleration = Vector3.ClampMagnitude(force, maxForce/4);
            return;
        }

        Vector3 fo = cohesionForce*10 + alignmentForce + obstacleAvoidanceForce;
        fo = Vector3.ClampMagnitude(fo, maxForce);
        
        acceleration = fo;
    }

    public void UpdatePositionPrediction(int numberOfTimeApplied)
    {
        for (int i = 0; i < numberOfTimeApplied; i++)
        {
            velocity += acceleration * 0.02f;
            velocity = Vector3.ClampMagnitude(velocity, maxSpeed);

            // Apply damping to reduce the velocity over time
            velocity *= dampingFactor;

            position += velocity * 0.02f;
            position.y = spawnHeight;
        }

        acceleration = Vector3.zero;
    }
}
