using UnityEngine;

public class Waypoint : MonoBehaviour {

	public Waypoint[] prevWaypoints;
	public Waypoint[] nextWaypoints;

	void OnDrawGizmos() {
		Gizmos.color = Color.cyan;
		Gizmos.DrawWireCube(transform.position, Vector3.one * .5f);

		DrawWaypoints(prevWaypoints, Color.magenta);
		DrawWaypoints(nextWaypoints, Color.green);
	}

	void DrawWaypoints(Waypoint[] waypoints, Color color) {
		if (waypoints == null || waypoints.Length == 0)
			return;

		Gizmos.color = color;

		for (int i = 0; i < waypoints.Length; i++) {
			Vector3 pointOverWaypoint = transform.TransformPoint(Vector3.up * 3f);

			if (waypoints == prevWaypoints)
				Gizmos.DrawLine(pointOverWaypoint, waypoints[i].transform.position);
			else {
				Gizmos.DrawLine(transform.position, pointOverWaypoint);
				Gizmos.DrawLine(pointOverWaypoint, waypoints[i].transform.position);
			}				
		}
	}
}
