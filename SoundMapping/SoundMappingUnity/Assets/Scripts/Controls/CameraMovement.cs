using System.Collections;
using System.Collections.Generic;
using System.Data.Common;
using FischlWorks_FogWar;
using Unity.VisualScripting;
using UnityEngine;

public class CameraMovement : MonoBehaviour
{
    public static Camera cam;
    public Transform swarmHolder;
    private int FOVDrones = 5;

    public float heightCamera = 10;

    public GameObject fogWarManager;

    public string state = "TDView";
    // Start is called before the first frame update

    public const float animationTime = 0.5f;

    public static GameObject embodiedDrone = null;
    public static GameObject nextEmbodiedDrone = null;
    public Quaternion intialCamRotation;
    const float DEFAULT_HEIGHT_CAMERA = 20;

    public float rotationSpeed = 80;

    void Start()
    {
        cam = Camera.main;
        intialCamRotation = cam.transform.rotation;

        this.GetComponent<sendInfoGameObject>().setupCallback(getCameraPositionDE);
        this.GetComponent<sendInfoGameObject>().setupCallback(getEmbodiedDrone);

        StartCoroutine(TDView());
        
    }

    public void resetFogExplorers()
    {
        fogWarManager.GetComponent<csFogWar>().fogRevealers.Clear();
        
        for (int i = 0; i < swarmHolder.childCount; i++)
        {
            fogWarManager.GetComponent<csFogWar>().AddFogRevealer(swarmHolder.GetChild(i).gameObject.transform, FOVDrones, true);
        }

    }

    public Vector3 getCameraPosition()
    {
        if (cam.enabled)
        {
            return cam.transform.position;
        }
        else
        {
            return embodiedDrone.transform.position;
        }
    }
    // Update is called once per frame
    void updateTDView()
    {
        //if scolling up
        float rightStickVertical = Input.GetAxis("JoystickRightVertical");

        heightCamera += rightStickVertical * Time.deltaTime * 10;

        float rightStickHorizontal = Input.GetAxis("JoystickRightHorizontal");

        // applz rotation to the camera with lerp
        cam.transform.Rotate(-Vector3.forward, rightStickHorizontal * Time.deltaTime * rotationSpeed);
    }

    void updateDroneView()
    {
        float rightStickHorizontal = Input.GetAxis("JoystickRightHorizontal");

        // applz rotation to the embodied drone with lerp
        embodiedDrone.transform.Rotate(Vector3.up, rightStickHorizontal * Time.deltaTime * rotationSpeed);

    }

    public IEnumerator TDView()
    {
        state = "TDView";
        yield return new WaitForSeconds(0.01f);
        try
        {       
            List<GameObject> drones = DroneNetworkManager.dronesInMainNetworkDistance;
            if (drones.Count > 0)
            {
                Vector3 center = Vector3.zero;
                foreach (GameObject drone in drones)
                {
                    center += drone.transform.position;
                }
                center /= drones.Count;

                center.y = DEFAULT_HEIGHT_CAMERA;
                cam.GetComponent<Camera>().orthographicSize = heightCamera;
                cam.transform.position = Vector3.Lerp(cam.transform.position, center, Time.deltaTime * 2);        

            }
        } catch
        {
            print("Error TDView");
        }
    
        updateTDView();
        if(embodiedDrone != null)
        {
            StartCoroutine(goAnimation(animationTime));
        }
        else
        {
            StartCoroutine(TDView());
        }
    }

    public IEnumerator goAnimationDroneToDrone(float _animationTime, GameObject lastEmbodiedDrone)
    {
        state = "goAnimationDrone";
        float elapsedTime = 0;
        //position of the active camera
        Vector3 startingPos = cam.transform.position;

        float startingFOV = lastEmbodiedDrone.GetComponent<Camera>().fieldOfView;
        while (elapsedTime < _animationTime)
        {
            lastEmbodiedDrone.GetComponent<Camera>().fieldOfView = Mathf.Lerp(lastEmbodiedDrone.GetComponent<Camera>().fieldOfView, 30, elapsedTime / _animationTime);
            //look at the next drone
            Vector3 direction = (embodiedDrone.transform.position - lastEmbodiedDrone.transform.position).normalized;
            lastEmbodiedDrone.transform.rotation = Quaternion.LookRotation(direction);
            elapsedTime += Time.deltaTime;
            yield return new WaitForEndOfFrame();
        }

        lastEmbodiedDrone.GetComponent<Camera>().enabled = false;
        lastEmbodiedDrone.GetComponent<Camera>().fieldOfView = startingFOV;

        embodiedDrone.GetComponent<Camera>().enabled = true;
        //look at the same direction as the last drone
        Vector3 direction2 = (embodiedDrone.transform.position - lastEmbodiedDrone.transform.position).normalized;
        embodiedDrone.transform.rotation = Quaternion.LookRotation(direction2);
        cam.enabled = false;

        StartCoroutine(droneView());
    }

    public IEnumerator goAnimation(float _animationTime)
    {
        state = "goAnimation";
        float elapsedTime = 0;
        //position of the active camera
        Vector3 startingPos = cam.transform.position;
        while (elapsedTime < _animationTime)
        {
            if(embodiedDrone == null) // if was in a drone view before 
            {
                cam.transform.rotation = intialCamRotation;
                StartCoroutine(TDView());
                yield break;
            }

            cam.GetComponent<Camera>().orthographicSize = Mathf.Lerp(cam.GetComponent<Camera>().orthographicSize, swarmModel.spawnHeight, elapsedTime / _animationTime);
            Vector3 positionToGo = new Vector3(embodiedDrone.transform.position.x, heightCamera, embodiedDrone.transform.position.z);
            cam.transform.position = Vector3.Lerp(cam.transform.position, positionToGo, elapsedTime / _animationTime);
            elapsedTime += Time.deltaTime;
            yield return new WaitForEndOfFrame();
        }

        embodiedDrone.GetComponent<Camera>().enabled = true;
        Vector3 direction = cam.transform.up;
        embodiedDrone.transform.rotation = Quaternion.LookRotation(direction);
        cam.enabled = false;

        StartCoroutine(droneView());
    }

    public IEnumerator droneView()
    {
        state = "droneView";
        Vector3 lastPosition = embodiedDrone.transform.position;
        GameObject lastEmbodiedDroneCam = embodiedDrone;
        while (embodiedDrone != null)
        {
            lastPosition = embodiedDrone.transform.position;
            updateDroneView();
            if(nextEmbodiedDrone != null)
            {
                GameObject lastEmbodiedDrone = embodiedDrone;
                setEmbodiedDrone(nextEmbodiedDrone);
                StartCoroutine(goAnimationDroneToDrone(animationTime, lastEmbodiedDrone ));
                yield break;
            }
            yield return new WaitForSeconds(0.01f);
        }

        cam.transform.position = new Vector3(lastPosition.x, heightCamera, lastPosition.z);
        cam.transform.rotation = intialCamRotation;
        Vector3 direction = lastEmbodiedDroneCam.transform.forward;
        direction.y = 0;
        cam.enabled = true;
        StartCoroutine(TDView());

    }

    public static void setEmbodiedDrone(GameObject drone)
    {
        embodiedDrone = drone;
        drone.GetComponent<DroneController>().droneFake.embodied = true;
        nextEmbodiedDrone = null;
    }

    public static void desembodiedDrone()
    {
        embodiedDrone.GetComponent<DroneController>().droneFake.embodied = false;
        embodiedDrone = null;
    }

    DataEntry getCameraPositionDE()
    {
        return new DataEntry("camera", cam.transform.position.x.ToString());
    }

    DataEntry getEmbodiedDrone()
    {
        if(embodiedDrone == null)
        {
            return new DataEntry("embodiedDrone", "null");
        }
        return new DataEntry("embodiedDrone", embodiedDrone.name);
    }
}
