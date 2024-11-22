using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
using UnityEngine.UI;
using System.IO;

public class Timer : MonoBehaviour
{
    // Existing variables
    public TextMeshProUGUI timerText;
    public TextMeshProUGUI timerTextNetwork;
    public TextMeshProUGUI droneDied;
    public TextMeshProUGUI leaderBoard;
    public TextMeshProUGUI HTTPserver;
    public TextMeshProUGUI TCPserver;
    public float elapsedTime = 0f;
    public float elapsedTimeNetwork = 0f;
    private Coroutine timerCoroutine;
    private Coroutine timerCoroutineNetwork;
    public int numberDroneDied = 0;

    public TMP_InputField nameInputField; // InputField for the player's name

    // Data structures for leaderboard
    [System.Serializable]
    public class LeaderboardEntry
    {
        public string name;
        public float time;
    }

    [System.Serializable]
    public class LeaderboardData
    {
        public List<LeaderboardEntry> entries = new List<LeaderboardEntry>();
    }

    public List<LeaderboardEntry> leaderboardEntries = new List<LeaderboardEntry>();

    void Start()
    {
        // Load leaderboard from file
        LoadLeaderboardFromFile();
        UpdateLeaderboardDisplay();

        // Initialize the timer display
        UpdateTimerDisplay();
        UpdateTimerDisplayNetwork();
    }

    void HideLeaderboard()
    {
        leaderBoard.text = "";
        nameInputField.enabled = false;
    }

    void ShowLeaderboard()
    {
        UpdateLeaderboardDisplay();
        nameInputField.enabled = true;
    }

    public void Restart()
    {
        elapsedTime = 0;
        elapsedTimeNetwork = 0;
        numberDroneDied = 0;
        UpdateTimerDisplay();
        UpdateTimerDisplayNetwork();

        StopTimerNetwork();
        StopTimer();

        ShowLeaderboard();

        droneDied.text = "Drone died : " + numberDroneDied.ToString();
    }

    public void StartTimer()
    {
        if (timerCoroutine == null)
        {
            HideLeaderboard();
            timerCoroutine = StartCoroutine(TimerCoroutine());
        }
    }

    public void DroneDiedCallback()
    {
        numberDroneDied++;
        droneDied.text = "Drone died : " + numberDroneDied.ToString();
        elapsedTime += 5;
    }

    public void StopTimer()
    {
        if (timerCoroutine != null)
        {
            StopCoroutine(timerCoroutine);
            timerCoroutine = null;
            SaveScore();

            Restart();
        }
    }

    public void SaveScore()
    {
        // Get the player's name from the InputField
        string playerName = nameInputField.text;

        // Create a new leaderboard entry
        LeaderboardEntry newEntry = new LeaderboardEntry
        {
            name = playerName,
            time = elapsedTime + elapsedTimeNetwork
        };

        // Add the new entry to the list
        leaderboardEntries.Add(newEntry);

        // Sort the list by time (ascending)
        leaderboardEntries.Sort((x, y) => x.time.CompareTo(y.time));

        // Keep only the top 10 entries
        if (leaderboardEntries.Count > 10)
        {
            leaderboardEntries.RemoveRange(10, leaderboardEntries.Count - 10);
        }

        // Update the leaderboard display
        UpdateLeaderboardDisplay();

        // Save the leaderboard to file
        SaveLeaderboardToFile();
    }

    public void SaveLeaderboardToFile()
    {
        LeaderboardData leaderboardData = new LeaderboardData
        {
            entries = leaderboardEntries
        };

        string json = JsonUtility.ToJson(leaderboardData, true);
        string path = Path.Combine(Application.persistentDataPath, "leaderboard.json");
        File.WriteAllText(path, json);
    }

    public void LoadLeaderboardFromFile()
    {
        string path = Path.Combine(Application.persistentDataPath, "leaderboard.json");

        if (File.Exists(path))
        {
            string json = File.ReadAllText(path);
            LeaderboardData leaderboardData = JsonUtility.FromJson<LeaderboardData>(json);
            leaderboardEntries = leaderboardData.entries;
        }
    }

    private void UpdateLeaderboardDisplay()
    {
        leaderBoard.text = "Leaderboard:\n";
        int rank = 1;
        foreach (var entry in leaderboardEntries)
        {
            leaderBoard.text += $"{rank}. {entry.name} : {entry.time:F2}\n";
            rank++;
        }
    }

    public void StartTimerNetwork()
    {
        if (elapsedTime == 0)
        {
            return;
        }
        if (timerCoroutineNetwork == null)
        {
            timerCoroutineNetwork = StartCoroutine(TimerCoroutineNetwork());
        }
    }

    public void StopTimerNetwork()
    {
        if (timerCoroutineNetwork != null)
        {
            StopCoroutine(timerCoroutineNetwork);
            timerCoroutineNetwork = null;
        }
    }

    private IEnumerator TimerCoroutine()
    {
        while (true)
        {
            elapsedTime += Time.deltaTime;
            UpdateTimerDisplay();
            yield return null;
        }
    }

    private IEnumerator TimerCoroutineNetwork()
    {
        while (true)
        {
            elapsedTimeNetwork += Time.deltaTime;
            UpdateTimerDisplayNetwork();
            yield return null;
        }
    }

    private void UpdateTimerDisplay()
    {
        timerText.text = elapsedTime.ToString("F2");
    }

    private void UpdateTimerDisplayNetwork()
    {
        timerTextNetwork.text = elapsedTimeNetwork.ToString("F2");
    }
}
