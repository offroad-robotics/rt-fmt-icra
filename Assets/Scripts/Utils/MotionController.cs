using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MotionController{

    public float gain = 0.0f;
    MonoBehaviour unityComponent;
    float executedCost;
    Vector3 lastPos;


    public MotionController(MonoBehaviour rtfmt, float gain)
    {
        this.gain = gain;
        this.unityComponent = rtfmt;
        this.lastPos = this.unityComponent.transform.position;
    }

    public void setGain(float gain)
    {
        this.gain = gain;
    }

    public void control(Vector3 setpoint, float clearance)
    {
        Vector3 error = (setpoint - this.unityComponent.transform.position);
        Vector3 velocityVector = this.gain * error.normalized;
        //Debug.Log("Setpoint: " + setpoint+ ", Vel vector: " + velocityVector +  ", Object position: " + this.unityComponent.transform.position);
        //Debug.Log(velocityVector.magnitude);
        if(error.magnitude > clearance)
        {
            this.unityComponent.GetComponent<Rigidbody>().velocity = velocityVector;
        }
        else
        {
            this.unityComponent.GetComponent<Rigidbody>().velocity = Vector3.zero;
            this.unityComponent.GetComponent<Rigidbody>().angularVelocity = Vector3.zero;

        }

        updateExecutedCost();
    }

    public void clearCost()
    {
        this.lastPos = this.unityComponent.transform.position;
        this.executedCost = 0;
    }
    public float getExecutedCost()
    {
        return this.executedCost;
    }
    public void updateExecutedCost()
    {
        Vector3 pos = this.unityComponent.transform.position;
        Vector3 dPos = pos - this.lastPos;
        
        float dist = dPos.magnitude;
        this.executedCost += dist;

        this.lastPos = pos;
    }

}
