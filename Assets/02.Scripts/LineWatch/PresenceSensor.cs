using System.Collections.Generic;
using UnityEngine;

namespace LineWatch
{
    [RequireComponent(typeof(BoxCollider))]
    public sealed class PresenceSensor : MonoBehaviour
    {
        private readonly HashSet<Collider> contacts = new HashSet<Collider>();

        public bool Occupied { get; private set; }
        public GameObject DetectedObject { get; private set; }
        public int EnterCount { get; private set; }
        public int ExitCount { get; private set; }

        private void Awake()
        {
            GetComponent<BoxCollider>().isTrigger = true;
        }

        private void OnTriggerEnter(Collider other)
        {
            Register(other);
        }

        private void OnTriggerStay(Collider other)
        {
            // Recover an overlap if this sensor was disabled and enabled inside it.
            Register(other);
        }

        private void Register(Collider other)
        {
            if (!isActiveAndEnabled || other.isTrigger || !contacts.Add(other)) return;
            RefreshState();
        }

        private void OnTriggerExit(Collider other)
        {
            if (contacts.Remove(other)) RefreshState();
        }

        private void FixedUpdate()
        {
            // Unity need not send OnTriggerExit when a collider is removed or disabled.
            if (contacts.RemoveWhere(IsUnavailable) > 0) RefreshState();
        }

        private static bool IsUnavailable(Collider contact)
        {
            return contact == null || !contact.enabled || !contact.gameObject.activeInHierarchy;
        }

        private void RefreshState()
        {
            contacts.RemoveWhere(IsUnavailable);
            bool occupied = contacts.Count > 0;
            DetectedObject = null;
            foreach (var contact in contacts)
            {
                DetectedObject = contact.attachedRigidbody != null
                    ? contact.attachedRigidbody.gameObject : contact.gameObject;
                break;
            }

            if (occupied == Occupied) return;
            Occupied = occupied;
            if (occupied) EnterCount++;
            else ExitCount++;
        }

        private void OnDisable()
        {
            contacts.Clear();
            RefreshState();
        }
    }
}
