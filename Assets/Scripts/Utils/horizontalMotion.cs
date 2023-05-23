using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class horizontalMotion : MonoBehaviour {
    // Use this for initialization
    public float speed = 1;
    float curSpeed;
    bool firstCollision = true;
    void Start()
    {
        Reset();
    }

    // Update is called once per frame
    void Update()
    {
        this.GetComponent<Rigidbody>().velocity = new Vector3(curSpeed, 0, 0);
        //if (this.transform.position.x > xLim.y)
        //{
        //    curSpeed = -speed;
        //    Debug.Log("GO Left");
        //}
        //else if (this.transform.position.x < xLim.x)
        //{
        //    curSpeed = speed;
        //}

    }

    void Reset()
    {
        if (this.transform.position.x < 0)
        {
            curSpeed = speed;
        }
        else
        {
            curSpeed = -speed;
        }
    }

    void OnCollisionEnter(Collision collision)
    {
        Debug.Log("Collision");
        if (firstCollision == false)
        {
            curSpeed = -curSpeed;
        }
        else
        {
            firstCollision = false;
        }


    }

}
