using UnityEngine;
using System.Collections;

public class BallController : MonoBehaviour {
	
	private float mouseposX;
	private Vector3 rayHitWorldPosition;
	public float speed = 0;
	public float maxSpeed = 10;
	public float acceleration = 10;
	public float deceleration = 10;
	
	public int mouseDeltaAverageFrames = 20;
	
	private int startFrameCounter = 0;
	private int frameCounterLoop = 0;
	private Vector3 mouseDelta = Vector3.zero;
	private Vector3[] mouseDeltaArray;
	private Vector3 mouseDeltaAverage = Vector3.zero;
	private Vector3 lastPos = Vector3.zero;

    public LayerMask layerMask;

	private bool mouseActive = false;
	void Start () {
		mouseDeltaArray = new Vector3[mouseDeltaAverageFrames];
		for(int i = 0; i < mouseDeltaAverageFrames; i++)
		{
			mouseDeltaArray[i] = Vector3.zero;
		}
	}
	
	Vector3 GetAverageDelta()
	{
		Vector3 averageDelta = Vector3.zero;
		int notZeroCnt = 0;
		foreach(Vector3 delta in mouseDeltaArray)
		{
			if(delta != Vector3.zero)
			{
				averageDelta += delta;
				notZeroCnt++;
			}
		}
		averageDelta = averageDelta/notZeroCnt;
		return averageDelta;
	}
	
	void OnMouseDown()
	{
		startFrameCounter = 0;
		mouseActive = true;
		lastPos = Input.mousePosition;
		gameObject.GetComponent<Rigidbody>().isKinematic = true;
		
	}
	void OnMouseUp()
	{
        Debug.Log("OnMouseUp");
        Debug.Log("Delta X before: " + GetAverageDelta().x);
        float deltaX = Mathf.Min(GetAverageDelta().x, 5f);
        float deltaY = Mathf.Min(GetAverageDelta().y, 10f);
        Debug.Log("Delta X 2: " + deltaX);
        deltaY = Mathf.Max(deltaY, 0f);
        
        
        Debug.Log("Delta X: " + deltaX);
        Debug.Log("Delta Y: " + deltaY);

		mouseActive = false;
		gameObject.GetComponent<Rigidbody>().isKinematic = false;
        gameObject.GetComponent<Rigidbody>().AddForce(new Vector3(deltaX * 10, 0.0f, deltaY * 100));

		GameObject.Find ("SceneManager").GetComponent<PinManager>().DetachPinCollider();
		Camera.main.enabled = false;
        GameObject.Find("DetachedCamera").GetComponent<Camera>().enabled = false;
		GameObject.Find ("BallCamera").GetComponent<Camera>().enabled = true;
        
	}

    public void Reset()
    {
        gameObject.GetComponent<Rigidbody>().velocity = new Vector3(0, 0, 0);
        gameObject.GetComponent<Rigidbody>().isKinematic = true;
        gameObject.transform.position = new Vector3(0.0f, 0.0f, -2.0f);
        GameObject.Find("BallCamera").GetComponent<Camera>().enabled = false;

        GameObject.Find("DetachedCamera").GetComponent<Camera>().enabled = true;
    }

	void FixedUpdate()
	{
		if (mouseActive)
		{
			mouseDelta = Input.mousePosition - lastPos;
			
			if(startFrameCounter < mouseDeltaAverageFrames)
			{
				mouseDeltaArray[startFrameCounter] = mouseDelta;
				startFrameCounter++;
			}
			else
			{
				mouseDeltaArray[frameCounterLoop] = mouseDelta;
				frameCounterLoop++;
				if(frameCounterLoop == mouseDeltaAverageFrames)
					frameCounterLoop = 0;
			}
			// raycast
			Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
			RaycastHit hit;
			//Debug.Log ("Velocity X: "+rigidbody.velocity.x);
			//Debug.Log ("Velocity Y: "+rigidbody.velocity.y);
			//Debug.Log ("Velocity Z: "+rigidbody.velocity.z);
			if (Physics.Raycast(ray, out hit, 100, layerMask))
			{
                Debug.DrawLine(ray.origin, hit.point);

                transform.position = new Vector3(hit.point.x, 0.3f, hit.point.z - 0.7f);
			}
			
			lastPos = Input.mousePosition;
		}
	}
	
}
