using UnityEngine;

namespace avoidance
{
    [CreateAssetMenu]
    public class AvoidanceCommon : ScriptableObject
    {
        public const float AVOID_DIST = 3.0f;
        public const float RAY_ANGLE = 90;
        public const float RAY_RANGE = 1;
        public const int NUM_RAYS = 7;
        //public const int NUM_RAYS = 17;

        public float maxAvoidForce;
        public float maxSeeAhead;
        public float maxVelocity;
        public float maxSpeed;
        public float maxForce;
        public float mass;
        public float radius;
        public float rotLerpSpeed;
    }
}
