using UnityEngine;
using System.Collections;

public class CarAIController : MonoBehaviour {

	[Header("Waypoint")]
	[SerializeField] Waypoint targetWaypoint;
	[SerializeField] float waypointDistanceThreshold = 10f;

	[Header("Sensors")]
	[SerializeField] LayerMask obstaclesLayerMask = -1;
	[SerializeField] LayerMask brakingSensorLayerMask = -1;

	[SerializeField] float frontRayOffset = 1.5f;
	[SerializeField] float sideRayOffset = .75f;

	[SerializeField] float frontRayDistance = 3f;
	[SerializeField] float frontAngularRayDistance = 12f;
	[SerializeField] float sideRayDistance = 4f;

	[SerializeField] float frontRayAngle = 20f;

	[Header("Respawn")]
	[SerializeField] float waitToRespawn = 5f;

	[SerializeField, Tooltip("If speed of the AI car will be equal of lower this value `Wait To Respawn` seconds, it will be considered as stuck")]
	float stuckSpeedThreshold = 15f;

	float stuckTimer = 0f;

	float SqrDistanceToWaypoint {
		get {
			return (targetWaypoint.transform.position - transform.position).sqrMagnitude;
		}
	}

	bool IsWaypointReached {
		get {
			return SqrDistanceToWaypoint <= waypointDistanceThreshold * waypointDistanceThreshold;
		}
	}

	Waypoint PrevWaypoint {
		get {
			return targetWaypoint.prevWaypoints.Length > 0 ? targetWaypoint.prevWaypoints[Random.Range(0, targetWaypoint.prevWaypoints.Length)] : null;
		}
	}

	Waypoint NextWaypoint {
		get {
			return targetWaypoint.nextWaypoints.Length > 0 ? targetWaypoint.nextWaypoints[Random.Range(0, targetWaypoint.nextWaypoints.Length)] : null;
		}
	}

	bool IsAvoidingObstacles {
		get {
			return !Mathf.Approximately(sensorsSensivity, 0f);
		}
	}

	bool IsOnLowSpeed {
		get {
			return carBehaviour.Speed <= stuckSpeedThreshold;
		}
	}

	bool IsStuck {
		get {
			return stuckTimer >= waitToRespawn;
		}
	}

	CarBehaviour carBehaviour;
	float sensorsSensivity;
	float xDirNormalized;

	void OnDrawGizmos() {
		if (targetWaypoint) {
			Gizmos.color = Color.yellow;
			Gizmos.DrawLine(transform.position, targetWaypoint.transform.position);
		}
	}

	void Start() {
		carBehaviour = GetComponent<CarBehaviour>();
	}

	void Update() {
		if (IsOnLowSpeed) {
			stuckTimer += Time.deltaTime;

			if (stuckTimer >= waitToRespawn)
				Respawn();
		}
		else
			stuckTimer = 0f;

		if (IsWaypointReached)
			targetWaypoint = NextWaypoint;
	}

	void FixedUpdate() {
		Torque();

		Steering();

		ApplySensors();

		carBehaviour.ApplySpeedLimit();
	}

	void Steering() {
		// Get direction to the target waypoint, don't take into account y axis of the target waypoint while steering
		Vector3 steerDirection = transform.InverseTransformPoint(targetWaypoint.transform.position.x, transform.position.y, targetWaypoint.transform.position.z);
		xDirNormalized = steerDirection.x / steerDirection.magnitude;

		float steering = xDirNormalized * carBehaviour.maxSteeringAngle;

		// Flip steering direction if moving backwards
		if (carBehaviour.LocalVelocity.z < 0f)
			steering *= -1f;

		carBehaviour.ApplySteering(steering);
	}

	void Torque() {
		// Apply default motor torque
		carBehaviour.ApplyMotorTorque(carBehaviour.maxMotorTorque);

		// If angle between forward direction of the car and the target waypoint is big enough and the car has some speed
		if (Mathf.Abs(xDirNormalized) >= .5f && carBehaviour.Speed >= 20f) {
			// Apply brake torque and reset motor torque to make steering more efficient
			carBehaviour.ApplyMotorTorque(0f);
			carBehaviour.ApplyBrakeTorque(carBehaviour.maxBrakeTorque);
		}
		else if (IsAvoidingObstacles) {
			carBehaviour.ApplyMotorTorque(carBehaviour.maxMotorTorque * .25f);
			carBehaviour.ApplyBrakeTorque(carBehaviour.maxBrakeTorque * .25f);
		}
	}

	bool SensorRaycast(Vector3 origin, Vector3 direction, float distance, LayerMask mask) {
		bool hitSomething = Physics.Raycast(origin, direction, distance, mask);

		Debug.DrawRay(origin, direction * distance, hitSomething ? Color.red : Color.white);

		return hitSomething;
	}

	void ApplySensors() {
		sensorsSensivity = 0f;

		// Front braking sensor
		if (SensorRaycast(transform.position + transform.forward * frontRayOffset, transform.forward, frontRayDistance, brakingSensorLayerMask))
			carBehaviour.ApplyBrakeTorque(carBehaviour.maxBrakeTorque);
		else
			carBehaviour.ApplyBrakeTorque(0f);

		// Front left angular sensor
		if (SensorRaycast(transform.position + (transform.forward * frontRayOffset) - (transform.right * sideRayOffset),
			Quaternion.AngleAxis(-frontRayAngle, transform.up) * transform.forward, frontAngularRayDistance, obstaclesLayerMask))
			sensorsSensivity += .5f;
		// Front right angular sensor
		else if (SensorRaycast(transform.position + (transform.forward * frontRayOffset) + (transform.right * sideRayOffset),
			Quaternion.AngleAxis(frontRayAngle, transform.up) * transform.forward, frontAngularRayDistance, obstaclesLayerMask))
			sensorsSensivity -= .5f;

		// Side left straight sensor
		if (SensorRaycast(transform.position - transform.right * sideRayOffset, -transform.right, sideRayDistance, obstaclesLayerMask))
			sensorsSensivity += .5f;
		// Side right straight sensor
		else if (SensorRaycast(transform.position + transform.right * sideRayOffset, transform.right, sideRayDistance, obstaclesLayerMask))
			sensorsSensivity -= .5f;

		// Try to avoid obstacles if any detected
		if (IsAvoidingObstacles)
			carBehaviour.ApplySteering(sensorsSensivity * carBehaviour.maxSteeringAngle);
	}

	void Respawn() {
		stuckTimer = 0f;

		// If previous waypoint exists, set position of the car to the previous waypoint position, otherwise set it to the target waypoint position
		transform.position = PrevWaypoint ? PrevWaypoint.transform.position : targetWaypoint.transform.position;

		transform.position += Vector3.up;
		transform.rotation = Quaternion.LookRotation(targetWaypoint.transform.position - transform.position);

		carBehaviour.Velocity = transform.forward * Random.Range(2f, 4f);
		carBehaviour.AngularVelocity = Vector3.zero;
	}
}
