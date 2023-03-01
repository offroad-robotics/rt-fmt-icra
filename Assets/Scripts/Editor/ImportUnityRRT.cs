using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.IO;


#if UNITY_EDITOR
using UnityEditor;
#endif

public class ImportUnityRRT : MonoBehaviour {
    //Editor stuff
#if UNITY_EDITOR
	// Helper for copying the built tracker dll to the plugins folder
	[MenuItem("RRT/Import DLL")]
#endif
	static void importDll()
	{
		if (!tryCopy("C:\\Kourosh\\Project\\aalto_games\\aaltogames\\WIP\\Apps\\UnityRRT\\dll\\UnityRRT.dll"))
		{
			//if (!tryCopy("C:\Kourosh\Project\aalto_games\aaltogames\WIP\Apps\UnityRRT\dll\UnityRRT.dll"))
			//{
				Debug.LogError("Cannot copy the RRT dll, check that you've built it!");
				return;
			//}
		}
        DirectoryInfo src = new DirectoryInfo("C:\\Kourosh\\Project\\aalto_games\\aaltogames\\WIP\\Apps\\UnityRRT\\");
        DirectoryInfo dst = new DirectoryInfo("Assets/Scripts/UnityRRT");

        CopyFiles(src, dst, true,"*.cs");
	    Debug.Log("Copied the RRT DLL");
	}

    static bool tryCopy(string dllName)
	{
		if (File.Exists(dllName))
		{
			File.Copy(dllName,"Assets/Plugins/UnityRRT.dll",true);
			return true;
		}
		return false;
	}
    static void CopyFiles(DirectoryInfo source,
                          DirectoryInfo destination,
                          bool overwrite,
                          string searchPattern)
    {
        FileInfo[] files = source.GetFiles(searchPattern);

        //this section is what's really important for your application.
        foreach (FileInfo file in files)
        {
            file.CopyTo(destination.FullName + "\\" + file.Name, overwrite);
        }
    }
    // Use this for initialization
	void Start () {
	
	}
	
	// Update is called once per frame
	void Update () {
	
	}
}
