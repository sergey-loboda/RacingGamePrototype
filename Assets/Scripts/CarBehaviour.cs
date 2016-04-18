using UnityEngine;
using System.Collections;

public class CarBehaviour : MonoBehaviour {

	[System.Serializable]
	class Axle {
		public bool motor;
		public bool braking;
		public bool steering;

		public WheelCollider leftWheelCol;
		public WheelCollider rightWheelCol;

		public Transform leftWheelVisual;
		public Transform rightWheelVisual;
	}

	[SerializeField] Axle[] axles;

	[Header("Torque")]
	public float maxMotorTorque = 1500f;
	public float maxBrakeTorque = 1500f;

	[Header("Speed Limit"), Range(20f, 300f)]
	public float maxSpeedKPH = 150f;

	[Header("Steering"), Range(15f, 45f)]
	public float maxSteeringAngle = 30f;

	[Header("Steering AI")]
	[SerializeField, Tooltip("Enable it on cars controlled by AI to make wheels rotate smoothly")]
	bool smoothSteering;

	[SerializeField, Tooltip("Determines rotation speed (smoothing) of the wheel")]
	float steerSmoothing = 5f;

	[Header("Other")]
	[SerializeField, Tooltip("Custom CoM position, set up it if necessary, or left it blank")]
	Vector3 centerOfMass;

	[SerializeField, Tooltip("How fast car will be rotated towards ground when it starts flipping")]
	float antiFlipSmoothing = 1f;

	[Header("Hit Forces")]
	[SerializeField, Tooltip("Force that will be aplied to lightweight objects on colliding with them")]
	float ligthHitForce = 50f;

	[SerializeField, Tooltip("Impulse force that will be aplied to heavy objects on colliding with them")]
	float heavyHitForce = 250f;

	[SerializeField, Tooltip("Impulse force that will be aplied to the car if it hits static object")]
	float selfHitForce = 100f;

	[SerializeField, Tooltip("If object's mass is greater or equal this value, it will be considered as heavy object")]
	float heavyWeightThreshold = 1000f;

	public float Speed {
		get {
			return speed;
		}
	}

	public Vector3 Velocity {
		get {
			return body.velocity;
		}
		set {
			body.velocity = value;
		}
	}

	public Vector3 LocalVelocity {
		get {
			return transform.InverseTransformDirection(body.velocity);
		}
	}

	public Vector3 AngularVelocity {
		get {
			return body.angularVelocity;
		}
		set {
			body.angularVelocity = value;
		}
	}

	Rigidbody body;
	float speed;

	void OnDrawGizmos() {
		Gizmos.color = Color.red;
		Gizmos.DrawWireSphere(transform.position + centerOfMass, .1f);
	}

	void Start() {
		body = GetComponent<Rigidbody>();

		if (centerOfMass != Vector3.zero)
			body.centerOfMass = centerOfMass;
	}

	void FixedUpdate() {
		// Velocity magnitude - speed of the rigidbody in meters per second, magnitude * 3.6f - speed in km per hour
		speed = body.velocity.magnitude * 3.6f;

		foreach (Axle axle in axles) {
			UpdateVisualWheels(axle.leftWheelCol, axle.leftWheelVisual);
			UpdateVisualWheels(axle.rightWheelCol, axle.rightWheelVisual);

			AntiFlipProtection(axle.leftWheelCol, axle.rightWheelCol);
		}
	}

	void OnCollisionEnter(Collision other) {
		Vector3 hitPoint = other.contacts[0].point;

		if (other.rigidbody) {
			// Apply force to other object only if it's not felt from above
			if (-other.relativeVelocity.y < .9f) {
				bool isHeavy = other.rigidbody.mass >= heavyWeightThreshold;

				// If other object is heavy enough, apply greater hit force to it
				float hitForce = isHeavy ? heavyHitForce : ligthHitForce;
				ForceMode forceMode = isHeavy ? ForceMode.Impulse : ForceMode.Force;

				other.rigidbody.AddForceAtPosition(-other.relativeVelocity * hitForce, hitPoint, forceMode);
			}
		}
		// If this car hit static object, apply hit force to the car
		else if (other.gameObject.layer != LayerMask.NameToLayer("Ground")) {
			body.AddForceAtPosition(other.relativeVelocity * selfHitForce, hitPoint, ForceMode.Impulse);
		}
	}

	public void ApplySteering(float steering) {
		foreach (Axle axle in axles) {
			if (axle.steering) {
				if (smoothSteering) {
					axle.leftWheelCol.steerAngle = Mathf.Lerp(axle.leftWheelCol.steerAngle, steering, steerSmoothing * Time.fixedDeltaTime);
					axle.rightWheelCol.steerAngle = Mathf.Lerp(axle.rightWheelCol.steerAngle, steering, steerSmoothing * Time.fixedDeltaTime);
				}
				else {
					axle.leftWheelCol.steerAngle = steering;
					axle.rightWheelCol.steerAngle = steering;
				}
			}
		}		
	}

	public void ApplyMotorTorque(float motorTorque) {
		foreach (Axle axle in axles) {
			if (axle.motor) {
				axle.leftWheelCol.motorTorque = motorTorque;
				axle.rightWheelCol.motorTorque = motorTorque;
			}
		}		
	}

	public void ApplyBrakeTorque(float brakeTorque) {
		foreach (Axle axle in axles) {
			if (axle.braking) {				
				axle.leftWheelCol.brakeTorque = brakeTorque;
				axle.rightWheelCol.brakeTorque = brakeTorque;
			}
		}		
	}

	public void ApplySpeedLimit() {
		if (Speed > maxSpeedKPH) {
			ApplyMotorTorque(0f);
			ApplyBrakeTorque(maxBrakeTorque);
		}
	}

	void AntiFlipProtection(WheelCollider leftWheelCol, WheelCollider rightWheelCol) {
		// Rotate car towards ground, if it's not grounded and rotated on x or z axes
		if (!(leftWheelCol.isGrounded | rightWheelCol.isGrounded) &&
			(Mathf.Abs(transform.eulerAngles.x) > 5f | Mathf.Abs(transform.eulerAngles.z) > 5f)) {
			Quaternion targetRot = transform.rotation;
			targetRot.x = targetRot.z = 0f;

			transform.rotation = Quaternion.Slerp(transform.rotation, targetRot, antiFlipSmoothing * Time.fixedDeltaTime);
		}
	}

	public void UpdateVisualWheels(WheelCollider wheelCol, Transform wheelVisual) {
		Vector3 pos;
		Quaternion rot;

		wheelCol.GetWorldPose(out pos, out rot);

		wheelVisual.transform.position = pos;
		wheelVisual.transform.rotation = rot;
	}
}
