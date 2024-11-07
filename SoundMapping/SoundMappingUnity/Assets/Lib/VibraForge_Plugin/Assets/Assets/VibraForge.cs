using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.VFX;
using System;
using UnityEditor.PackageManager;

[Serializable]
public class VibraForge : MonoBehaviour
{
    private static TcpSender sender;
    private static Dictionary<string, int> command;

    void Start()
    {
        sender = this.GetComponent<TcpSender>();
        command = new Dictionary<string, int>()
        {
            { "addr", -1 },
            { "mode", 0 },
            { "duty", 0 },
            { "freq", 2 }
        };
    }

    public static string DictionaryToString(Dictionary<string, int> dictionary)
    {
        string dictionaryString = "{";
        foreach (KeyValuePair<string, int> keyValues in dictionary)
        {
            dictionaryString += "\"" + keyValues.Key + "\": " + keyValues.Value + ", ";
        }
        return dictionaryString.TrimEnd(',', ' ') + "}";
    }

    public static void SendCommand(int addr, int mode, int duty, int freq)
    {
        command["addr"] = addr;
        command["mode"] = mode;
        command["duty"] = duty;
        command["freq"] = freq;
        sender.SendData(DictionaryToString(command));
        print("Command sent: " + DictionaryToString(command));
    }


    //on quit
    void OnApplicationQuit()
    {
        for(int i = 0; i < 10; i++)
        {
            SendCommand(i, 0, 0, 0);
        }
        
        //wait for 1 second
        System.Threading.Thread.Sleep(1000);

        //close the socket
        sender.client.Close();
    }
}
