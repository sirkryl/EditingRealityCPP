using UnityEngine;
using UnityEngine.UI;
using System.Collections;

public class ScoreManager : MonoBehaviour
{
	public Text scoreText;
	public Text strikeText;
    public Text throwText;

	private int score = 0;
    private int throwCount = 0;
	private int objectCount = 0;
	// Use this for initialization
	void Start ()
	{

	}

	public void SetObjectCount(int _objectCount)
	{
		objectCount = _objectCount;
	}

    public void IncreaseThrowCount()
    {
        throwCount++;
        throwText.text = "THROWS: " + throwCount.ToString();
    }

	public void PinDown()
	{
		score++;
        scoreText.text = score.ToString() + "/" + objectCount.ToString();
		if(score == objectCount)
			strikeText.enabled = true;
	}

	public void Reset()
	{
		score = 0;
        throwCount = 0;
		strikeText.enabled = false;
		scoreText.text = score.ToString () + "/" + objectCount.ToString();
        throwText.text = "THROWS: "+throwCount.ToString();
	}

	// Update is called once per frame
	void Update ()
	{

	}
}

