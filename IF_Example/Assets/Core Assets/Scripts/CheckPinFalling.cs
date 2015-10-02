using UnityEngine;
using System.Collections;

public class CheckPinFalling : MonoBehaviour
{

	GameObject sceneManager;
	public GameObject ownPin;
	// Use this for initialization
	void Start ()
	{
		sceneManager = GameObject.Find ("SceneManager");
	}

	// Update is called once per frame
	void Update ()
	{

	}

	void OnTriggerExit(Collider other)
	{
		if(other.gameObject == ownPin)
		{
			Debug.Log ("It's my pin!");
			sceneManager.GetComponent<ScoreManager>().PinDown();
			Destroy(gameObject);
		}
	}
}

