using UnityEngine;

public class Destructible : MonoBehaviour {

	[SerializeField] GameObject destroyedPrefab;

	void OnTriggerEnter(Collider other) {
		if (!other.GetComponentInParent<CarBehaviour>())
			return;

		Transform parent = transform.parent;
		(Instantiate(destroyedPrefab, parent.position, parent.rotation) as GameObject).transform.parent = transform.root;
		Destroy(parent.gameObject);
	}
}
