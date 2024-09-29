using Unity.Physics;
using Unity.Collections;
using UnityEngine.Assertions;

namespace avoidance.dots
{
    public struct MousePickCollector : ICollector<Unity.Physics.RaycastHit>
    {
        public bool ignoreTriggers;
        public bool ignoreStatic;
        public NativeArray<RigidBody> rigidBodies;
        public int numDynamicBodies;

        public bool EarlyOutOnFirstHit => false;
        public float MaxFraction { get; private set; }
        public int NumHits { get; private set; }

        public RaycastHit hit => _closesHit;

        private RaycastHit _closesHit;
        private bool _overlap;

        public MousePickCollector(float maxFraction, NativeArray<RigidBody> rigidBodies, int numDynamicBodies)
        {
            _closesHit = default;
            MaxFraction = maxFraction;
            NumHits = 0;

            ignoreTriggers = true;
            ignoreStatic = true;
            _overlap = false;

            this.rigidBodies = rigidBodies;
            this.numDynamicBodies = numDynamicBodies;
        }

        public bool AddHit(Unity.Physics.RaycastHit hit)
        {
            Assert.IsTrue(hit.Fraction <= MaxFraction);

            if (_overlap)
                return false;

            _overlap = rigidBodies[hit.RigidBodyIndex].CustomTags != (1u << CustomTag.Floor);

            bool isAcceptable = true;

            if (ignoreStatic)
                isAcceptable = isAcceptable && (hit.RigidBodyIndex >= 0) && (hit.RigidBodyIndex < numDynamicBodies);

            if (ignoreTriggers)
                isAcceptable = isAcceptable && hit.Material.CollisionResponse != CollisionResponsePolicy.RaiseTriggerEvents;

            if (!isAcceptable)
                return false;

            MaxFraction = hit.Fraction;
            _closesHit = hit;
            NumHits = 1;

            return true;
        }
    }
}

