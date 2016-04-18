using UnityEngine;

public class CameraTargetSwitcher : MonoBehaviour {	

	[SerializeField] Transform[] targets;

	FollowCamera cam;
	int curTargetIndex;
	
	void Start () {
		cam = GetComponent<FollowCamera>();

		curTargetIndex = System.Array.IndexOf(targets, cam.target);
	}

	void Update () {
		// Set previous target
		if (Input.GetKeyDown(KeyCode.Q)) {
			SetTarget(curTargetIndex - 1);
		}
		// Set next target
		else if (Input.GetKeyDown(KeyCode.E)) {
			SetTarget(curTargetIndex + 1);
		}
	}

	void SetTarget(int newTargetIndex) {
		if (newTargetIndex < 0)
			newTargetIndex = targets.Length - 1;

		newTargetIndex %= targets.Length;

		curTargetIndex = newTargetIndex;

		cam.target = targets[curTargetIndex];
	}
}
