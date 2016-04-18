using UnityEngine;

public class FollowCamera : MonoBehaviour {
	
	public Transform target;

	[SerializeField] Vector3 positionOffset = new Vector3(0f, 3f, 6f);
	[SerializeField] Vector3 lookAtOffset = Vector3.zero;

	[SerializeField] float dampingTime = 5f;

	[SerializeField] bool followBehind = true;
	[SerializeField] bool followTilt;

	[SerializeField] bool smoothRotation = true;
	[SerializeField] float rotationDamping = 10f;

	Vector3 velocity;

	void OnDrawGizmos() {
		Gizmos.color = Color.yellow;
		Gizmos.DrawWireCube(target.TransformPoint(lookAtOffset), Vector3.one * .25f);
	}

	void FixedUpdate() {
		followBehind = !Input.GetKey(KeyCode.B);

		// Position setup
		float distance = followBehind ? -positionOffset.z : positionOffset.z;
		Vector3 targetPosition = target.TransformPoint(positionOffset.x, positionOffset.y, distance);
		transform.position = Vector3.SmoothDamp(transform.position, targetPosition, ref velocity, dampingTime * Time.fixedDeltaTime);

		// Rotation setup
		Vector3 upwardsDirection = followTilt ? target.up : Vector3.up;
		Quaternion targetRotation = Quaternion.LookRotation(target.TransformPoint(lookAtOffset) - transform.position, upwardsDirection);

		if (smoothRotation) {
			transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, rotationDamping * Time.fixedDeltaTime);
		}
		else {
			transform.rotation = targetRotation;
		}
	}
}
