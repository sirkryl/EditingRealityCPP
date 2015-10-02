using UnityEngine;
using System.Collections;
using UnityEngine.UI;

public class CountPins : MonoBehaviour {

	public Text score;
	public Text strike;
	private int scoreCnt = 0;
	private int objectCount = 0;

	public GameObject[] fallenPins;

	void Start()
	{

	}

	public void SetObjectCount (int _objectCount)
	{
		objectCount = _objectCount;
		fallenPins = new GameObject[objectCount];

	}

	void OnTriggerEnter (Collider other)
	{
		Debug.Log ("Object entered the trigger");
	}

	void OnTriggerStay (Collider other)
	{

	}

	void OnTriggerExit(Collider other)
	{
		Debug.Log ("Object exited the trigger");
		if(other.gameObject.tag == "Pin")
		{
			bool found = false;
			int cnt = 0;
			foreach(GameObject pin in fallenPins)
			{
				if (pin == other.gameObject)
					found = true;
				if(pin != null)
					cnt++;
			}
			if (!found)
			{
				fallenPins[cnt] = other.gameObject;
				Debug.Log ("It's a pin!");
				scoreCnt++;
				score.text = scoreCnt.ToString ();
				if(scoreCnt == objectCount)
					strike.enabled = true;
			}
		}
	}
}
