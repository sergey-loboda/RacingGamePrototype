using UnityEngine;

public class CarUserController : MonoBehaviour {

	[Header("Controls")]
	[SerializeField] string gasReverseAxisName = "Vertical";
	[SerializeField] string steeringAxisName = "Horizontal";
	[SerializeField] KeyCode brakeKey = KeyCode.Space;
	[SerializeField] KeyCode nitroKey = KeyCode.LeftShift;
	[SerializeField] KeyCode respawnKey = KeyCode.R;

	CarBehaviour carBehaviour;

	void Start() {
		carBehaviour = GetComponent<CarBehaviour>();
	}

	void Update() {
		if (Input.GetKeyDown(respawnKey))
			Respawn();
	}

	void FixedUpdate() {
		float motor = Input.GetAxis(gasReverseAxisName) * carBehaviour.maxMotorTorque;
		float steering = Input.GetAxis(steeringAxisName) * carBehaviour.maxSteeringAngle;

		// Nitro
		if (Input.GetKey(nitroKey)) {
			motor *= 4f;
		}
		// Fast start
		else if (carBehaviour.Speed < 30f) {
			motor *= 2f;
		}

		carBehaviour.ApplyMotorTorque(motor);
		carBehaviour.ApplySteering(steering);

		if (Input.GetKey(brakeKey)) {
			carBehaviour.ApplyBrakeTorque(carBehaviour.maxBrakeTorque);
			carBehaviour.ApplyMotorTorque(0f);
		}
		else {
			carBehaviour.ApplyBrakeTorque(0f);				
		}

		carBehaviour.ApplySpeedLimit();
	}

	void Respawn() {
		// Reset position and rotation
		transform.position += Vector3.up;
		transform.eulerAngles = new Vector3(0f, transform.eulerAngles.y, 0f);

		// Give little forward velocity to the car and reset angular velocity
		carBehaviour.Velocity = transform.forward * Random.Range(2f, 4f);
		carBehaviour.AngularVelocity = Vector3.zero;
	}
}
