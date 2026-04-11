using UnityEngine;
using System.Net.Sockets;
using System.IO;
using Newtonsoft.Json.Linq;

public class MMGStateReceiver : MonoBehaviour
{
    public Transform yachtRoot;
    public Transform rudderPivot;
    public PropellerVisual propellers;

    TcpClient client;
    StreamReader reader;

    void Start()
    {
        client = new TcpClient("127.0.0.1", 5005);
        reader = new StreamReader(client.GetStream());
    }

    void Update()
    {
        if (client.Available > 0)
        {
            string line = reader.ReadLine();
            if (line == null) return;

            JObject data = JObject.Parse(line);

            // Position
            float x = (float)data["x"];
            float z = (float)data["y"];   // Python y = Unity z
            yachtRoot.position = new Vector3(x, 0, z);

            // Heading
            float heading = (float)data["heading"];
            yachtRoot.rotation = Quaternion.Euler(0, heading, 0);

            // Rudder
            float rudderAngle = (float)data["rudder_angle"];
            rudderPivot.localRotation = Quaternion.Euler(0, rudderAngle, 0);

            // Propellers
            float rpmL = (float)data["rpm_left"];
            float rpmR = (float)data["rpm_right"];
            propellers.SetRPM(rpmL, rpmR);
        }
    }
}