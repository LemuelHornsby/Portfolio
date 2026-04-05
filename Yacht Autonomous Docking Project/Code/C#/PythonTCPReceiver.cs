using UnityEngine;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using Newtonsoft.Json.Linq;

public class PythonTCPReceiver : MonoBehaviour
{
    [Header("Ship Motion")]
    public YachtPoseApplier poseApplier;
    public YachtControlSurface controlSurface;

    [Header("Visual Animation")]
    public PropellerVisual propellerVisual;
    public RudderVisual rudderVisual;
    public EngineSoundController engineSoundController;   // NEW

    [Header("Connection Settings")]
    public string host = "127.0.0.1";
    public int port = 5005;

    private TcpClient client;
    private NetworkStream stream;
    private Thread receiveThread;
    private bool running = false;

    void Start()
    {
        ConnectToPython();
    }

    void OnApplicationQuit()
    {
        running = false;
        stream?.Close();
        client?.Close();
        receiveThread?.Abort();
    }

    void ConnectToPython()
    {
        receiveThread = new Thread(() =>
        {
            try
            {
                Debug.Log("Unity TCP Client: Connecting to Python...");

                client = new TcpClient(host, port);
                stream = client.GetStream();
                running = true;

                Debug.Log("Unity TCP Client: Connected to Python!");

                byte[] buffer = new byte[2048];

                while (running)
                {
                    int bytesRead = stream.Read(buffer, 0, buffer.Length);
                    if (bytesRead == 0)
                        continue;

                    string msg = Encoding.UTF8.GetString(buffer, 0, bytesRead);
                    string[] packets = msg.Split('\n');

                    foreach (string packet in packets)
                    {
                        if (string.IsNullOrWhiteSpace(packet))
                            continue;

                        ProcessJSON(packet);
                    }
                }
            }
            catch (System.Exception e)
            {
                Debug.Log("Unity TCP Client Error: " + e.Message);
            }
        });

        receiveThread.IsBackground = true;
        receiveThread.Start();
    }

    void ProcessJSON(string json)
    {
        try
        {
            JObject data = JObject.Parse(json);

            // REQUIRED FIELDS
            float x = data["x"] != null ? (float)data["x"] : 0f;
            float y = data["y"] != null ? (float)data["y"] : 0f;
            float psi = data["psi"] != null ? (float)data["psi"] : 0f;

            // OPTIONAL FIELDS
            float throttle = data["throttle"] != null ? (float)data["throttle"] : 0f;
            float rudder = data["rudder_angle"] != null ? (float)data["rudder_angle"] : 0f;

            UnityMainThreadDispatcher.Enqueue(() =>
            {
                // 1. Move the ship
                poseApplier.ApplyPose(x, y, psi);

                // 2. Update control logic
                controlSurface.SetControl(throttle, rudder);

                // 3. Animate propellers
                if (propellerVisual != null)
                {
                    float maxRPM = 3000f;     // visual multiplier
                    float rpm = throttle * maxRPM;

                    propellerVisual.SetRPM(rpm, rpm);

                    // 4. Send RPM to engine sound
                    if (engineSoundController != null)
                        engineSoundController.rpm = rpm;
                }

                // Rudder animation happens automatically in RudderVisual.Update()
            });
        }
        catch (System.Exception e)
        {
            Debug.Log("JSON parse error: " + e.Message + " | RAW: " + json);
        }
    }
}