using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class PinManager : MonoBehaviour {

	public GameObject emptyPrefabWithMeshRenderer;

	private GameObject[] pins;
	private Vector3[] pinPositions;
	private Quaternion[] pinRotations;
	private GameObject[] colliders;

	public bool loadMesh;
	string directory = "mesh/";
	GameObject spawnedPrefab;
	private int objectCount = 0;
	// Use this for initialization
	void Start () {
		if(loadMesh)
		{
			pins = new GameObject[256];
			colliders = new GameObject[256];
			pinPositions = new Vector3[256];
			pinRotations = new Quaternion[256];
			ObjImporter objImporter = new ObjImporter();
			string fileName = "object_";
			string meshPath;
			int index = 0;
			Debug.Log (directory+fileName+index.ToString ());
			while (System.IO.File.Exists(directory+fileName+index.ToString ()+".obj"))
			{
				meshPath = directory+fileName+index.ToString ()+".obj";
				Mesh importedMesh=objImporter.ImportFile(meshPath);
				spawnedPrefab=Instantiate(emptyPrefabWithMeshRenderer, new Vector3(transform.position.x,transform.position.y,transform.position.z),transform.rotation) as GameObject;
				spawnedPrefab.GetComponent<MeshFilter>().mesh = importedMesh;
				spawnedPrefab.AddComponent<CapsuleCollider>();
				pins[index] = spawnedPrefab;
				pinPositions[index] = spawnedPrefab.transform.position;
				pinRotations[index] = spawnedPrefab.transform.rotation;
				GameObject pinCollider = new GameObject();
				pinCollider.AddComponent<BoxCollider>();
				pinCollider.AddComponent<CheckPinFalling>();
				pinCollider.GetComponent<CheckPinFalling>().ownPin = spawnedPrefab;
				pinCollider.GetComponent<Collider>().transform.localScale = new Vector3(1.0f, 0.1f, 1.0f);
				pinCollider.GetComponent<Collider>().isTrigger = true;
				Vector3 prefabPosition = spawnedPrefab.transform.GetComponent<Renderer>().bounds.center;
				Vector3 prefabTop = spawnedPrefab.transform.GetComponent<Renderer>().bounds.max;
				pinCollider.transform.position = new Vector3(prefabPosition.x,prefabTop.y,prefabPosition.z);
				pinCollider.transform.parent = spawnedPrefab.transform;
				colliders[index] = pinCollider;
				index++;
				objectCount++;
			}
		}
		else
		{
			Debug.Log ("else");
			objectCount = 10;
		}
		GetComponent<ScoreManager>().SetObjectCount(objectCount);




	}

    public void Reset()
    {
        ResetPins();

        GetComponent<ScoreManager>().Reset();
        GameObject.Find("Ball").GetComponent<BallController>().Reset();
    }

    public void NextRoll()
    {
        GameObject.Find("Ball").GetComponent<BallController>().Reset();
        GetComponent<ScoreManager>().IncreaseThrowCount();
    }

	// Update is called once per frame
	void Update () {
		if (Input.GetKeyDown (KeyCode.R))
		{
            Reset();
		}
        if (Input.GetKeyDown (KeyCode.Space))
        {
            NextRoll();
        }
	}

	public void DetachPinCollider()
	{
		if(!loadMesh)
			return;
        if (colliders.Length == 0)
            return;
		for(int i = 0; i < objectCount; i++)
		{
            if(colliders[i] != null)
			    colliders[i].transform.parent = null;
		}
	}

	public void ResetPins()
	{
        for (int i = 0; i < objectCount; i++)
        {
            Destroy(colliders[i]);
        }
        for (int i = 0; i < objectCount; i++)
        {
            Destroy(pins[i]);
        }
        objectCount = 0;
        Start();
        return;
		if(!loadMesh)
			return;
		for(int i = 0; i < objectCount; i++)
		{
			Destroy(colliders[i]);
		}
		for(int i = 0; i < objectCount; i++)
		{
			pins[i].transform.position = pinPositions[i];
			pins[i].transform.rotation = pinRotations[i];

			GameObject pinCollider = new GameObject();
			pinCollider.AddComponent<BoxCollider>();
			pinCollider.AddComponent<CheckPinFalling>();
			pinCollider.GetComponent<CheckPinFalling>().ownPin = pins[i];
			pinCollider.GetComponent<Collider>().transform.localScale = new Vector3(1.0f, 0.1f, 1.0f);
			pinCollider.GetComponent<Collider>().isTrigger = true;
			Vector3 prefabPosition = pins[i].transform.GetComponent<Renderer>().bounds.center;
			Vector3 prefabTop = pins[i].transform.GetComponent<Renderer>().bounds.max;
			pinCollider.transform.position = new Vector3(prefabPosition.x,prefabTop.y,prefabPosition.z);
			pinCollider.transform.parent = pins[i].transform;
			colliders[i] = pinCollider;
		}
	}


}
