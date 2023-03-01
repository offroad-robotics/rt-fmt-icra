using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


public class RandomMotion : MonoBehaviour {

	Vector3 direction;
	System.Random rand;
	public int seed = 1;

	public float smoothingFactor = 0.5f;

	public float speed = 1;

	Vector3 oldVel;
    Vector3 curVel;

	Vector3 initialPos;
	// Use this for initialization
	void Start () {
		rand = new System.Random(seed);
		initialPos = transform.position;
	}
	
	// Update is called once per frame
	void Update () {

		oldVel = curVel;

		curVel = (1 - smoothingFactor) * randomVector(1, 1000) + smoothingFactor * oldVel;

		curVel = speed*(curVel/curVel.magnitude);
		//Debug.Log(curVel.magnitude);
		this.GetComponent<Rigidbody>().velocity = curVel;
	}


	Vector3 randomVector(float maxSpeed, int resolution)
    {
		Vector3 vector;
		float vx = (((float)rand.Next(0, 2 * resolution) - resolution) / resolution) * maxSpeed;
		float vz = (((float)rand.Next(0, 2 * resolution) - resolution) / resolution) * maxSpeed;

		vector = new Vector3(vx, 0, vz);

		
		return vector;
	}
	void Reset()
	{
        //Debug.Log("Reset called");
		transform.position = initialPos;
		GetComponent<Rigidbody>().velocity = Vector3.zero;
		GetComponent<Rigidbody>().angularVelocity = Vector3.zero;

	}
}
