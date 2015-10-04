using UnityEngine;
using System;
using System.Collections;
using System.IO.Ports;

public class SerialTest : MonoBehaviour {

	SerialPort stream = new SerialPort("/dev/tty.usbmodem1411", 115200);

	public float smooth = 2.0F;
	// Use this for initialization
	void Start () {
		stream.Open(); //Open the Serial Stream.
		Debug.Log(this);
		
		//set(90f, 90f, 90f);
	}
	
	// Update is called once per frame
	void Update () {
		if (stream.IsOpen)
		{
			try
			{
				string lines = stream.ReadLine();
				string[] values = lines.Split(' ');
//				float yaw = float.Parse(values[0]);
//				float pitch = float.Parse(values[1]);
//				float roll = float.Parse(values[2]);
//				Debug.Log(yaw + " " + pitch + " " + roll);
				
//				set(yaw, pitch, roll);

				transform.rotation = new Quaternion(float.Parse(values[1]), float.Parse(values[0]), float.Parse(values[2]), float.Parse(values[3]));
				if (stream.BytesToRead > 0 ) 
				{
//					Debug.Log(stream.BytesToRead);
					while(stream.BytesToRead > 50) {
						stream.ReadLine();
					}
				}
			}
			catch (Exception e)
			{
				Debug.Log(e.GetBaseException());
			}
		}
	}
	
	void set(float yaw, float pitch, float roll)
	{
		Quaternion target = Quaternion.Euler(-pitch, yaw, -roll);
		transform.rotation = Quaternion.Slerp(transform.rotation, target, Time.time * smooth);
	}
}
