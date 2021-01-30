using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GyroSensor : MonoBehaviour
{
    private bool gyroEnabled;
    private Gyroscope gyro;

    private GameObject cameraContainer;
    private Quaternion rot;
    // Start is called before the first frame update
    void Start()
    {
        cameraContainer = new GameObject("Camera Container");
        cameraContainer.transform.position = gameObject.transform.position;
        gameObject.transform.SetParent(cameraContainer.transform);

        gyroEnabled = EnableGyro();

    }

    private bool EnableGyro()
    {
        if (SystemInfo.supportsGyroscope)
        {
            gyro = Input.gyro;
            gyro.enabled = true;

            cameraContainer.transform.rotation = Quaternion.Euler(90f, -90f, 0f);
            rot = new Quaternion(0, 0, 1, 0);

            return true;
        }

        return false;
    }

    // Update is called once per frame
    void Update()
    {
        if (gyroEnabled)
        {
            gameObject.transform.localRotation = gyro.attitude * rot;
            //print(gyro.attitude);
            print(gameObject.transform.rotation);
        }
    }
}
