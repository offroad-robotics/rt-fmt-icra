using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class linearMotion : MonoBehaviour {

	List<float> xPosList = new List<float>{27.0f, 9f, -9f, -27.0f};
	List<float> zPosList = new List<float>{ -54f, -27f, 0f, 27.0f, 54f };
	// Use this for initialization
	public float speed = 1;
	float curSpeed;

	void Start () {
		Reset();
	}
	
	// Update is called once per frame
	void Update () {
		this.GetComponent<Rigidbody>().velocity = new Vector3(0, 0, curSpeed);
	}

	void Reset()
    {
		int xIdx = Random.Range(0, 4);
		int zIdx = Random.Range(0, 5);
		//this.transform.position = new Vector3(xPosList[xIdx], 0.5f, zPosList[zIdx]);
		Vector3 pos = this.transform.position;
		pos.z = zPosList[zIdx];
		this.transform.position = pos;
		if (this.transform.position.z < 0)
		{
			curSpeed = speed;
		}
        else
        {
			curSpeed = -speed;
        }
	}
}
