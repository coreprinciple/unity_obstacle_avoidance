using UnityEngine;
using Unity.Jobs;
using Unity.Burst;
using Unity.Physics;
using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Physics.Systems;
using static Unity.Physics.Math;

namespace avoidance.dots
{
    public struct MousePick : IComponentData
    {
        public bool ignoreTriggers;
        public bool ignoreStatic;
    }

    [DisallowMultipleComponent]
    public class MousePickAuthoring : MonoBehaviour
    {
        public bool ignoreTriggers = true;
        public bool ignoreStatic = true;

        protected void OnEnable() {}
    }

    class MousePickBaker : Baker<MousePickAuthoring>
    {
        public override void Bake(MousePickAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);

            AddComponent(entity, new MousePick() {
                ignoreTriggers = authoring.ignoreTriggers,
                ignoreStatic = authoring.ignoreStatic,
            });
        }
    }

    [UpdateInGroup(typeof(AfterPhysicsSystemGroup))]
    public partial class MousePickSystem : SystemBase
    {
        public const float k_MaxDistance = 100.0f;
        public NativeReference<PickData> pickDataRef;

        public JobHandle? pickJobHandle;

        public struct PickData
        {
            public float3 pickPoint;
            public bool picked;
        }

        [BurstCompile]
        struct Pick : IJob
        {
            [ReadOnly] public CollisionWorld collisionWorld;

            public NativeReference<PickData> pickDataRef;
            public RaycastInput rayInput;
            public float near;
            public float3 forward;
            
            [ReadOnly] public bool ignoreTriggers;
            [ReadOnly] public bool ignoreStatic;

            public void Execute()
            {
                var mousePickCollector = new MousePickCollector(1.0f, collisionWorld.Bodies, collisionWorld.NumDynamicBodies);
                mousePickCollector.ignoreTriggers = ignoreTriggers;
                mousePickCollector.ignoreStatic = ignoreStatic;

                bool picked = collisionWorld.CastRay(rayInput, ref mousePickCollector);

                if (!picked)
                {
                    pickDataRef.Value = new PickData();
                    return;
                }

                RigidBody hitBody = collisionWorld.Bodies[mousePickCollector.hit.RigidBodyIndex];

                if (hitBody.CustomTags != (1u << CustomTag.Floor) || mousePickCollector.NumHits > 1)
                    return;

                pickDataRef.Value = new PickData
                {
                    picked = true,
                    pickPoint = Mul(Inverse(new MTransform(hitBody.WorldFromBody)), mousePickCollector.hit.Position)
                };
            }
        }

        public MousePickSystem()
        {
            pickDataRef = new NativeReference<PickData>(Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            pickDataRef.Value = new PickData();
        }

        protected override void OnCreate()
        {
            RequireForUpdate<MousePick>();
        }

        protected override void OnUpdate()
        {
            if (Input.GetMouseButtonDown(0) && (Camera.main != null))
            {
                Vector2 mousePosition = Input.mousePosition;
                UnityEngine.Ray unityRay = Camera.main.ScreenPointToRay(mousePosition);
                var world = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;

                CollisionFilter filter = new CollisionFilter
                {
                    BelongsTo = 0xffffffff,
                    CollidesWith = 1u << 0,
                    GroupIndex = 0
                };

                Dependency = new Pick
                {
                    collisionWorld = world.CollisionWorld,
                    pickDataRef = pickDataRef,
                    
                    rayInput = new RaycastInput
                    {
                        Start = unityRay.origin,
                        End = unityRay.origin + unityRay.direction * k_MaxDistance,
                        Filter = filter
                    },

                    near = Camera.main.nearClipPlane,
                    forward = Camera.main.transform.forward,
                    ignoreTriggers = SystemAPI.GetSingleton<MousePick>().ignoreTriggers,
                    ignoreStatic = SystemAPI.GetSingleton<MousePick>().ignoreStatic,
                }.Schedule(Dependency);

                pickJobHandle = Dependency;
            }

            if (Input.GetMouseButtonUp(0))
            {
                if (pickJobHandle != null)
                    CompletePickJob();

                pickDataRef.Value = new PickData();
            }
        }

        private void CompletePickJob()
        {
            pickJobHandle.Value.Complete();

            if (pickDataRef.Value.picked)
                ApplyPick();
        }

        private void ApplyPick()
        {
            var testData = SystemAPI.GetSingleton<AgentTestDataComponent>();
            var entity = SystemAPI.GetSingletonEntity<AgentTestDataComponent>();

            testData.targetPosition = pickDataRef.Value.pickPoint;
            testData.isDirty = true;

            SystemAPI.SetComponent(entity, testData);
        }

        protected override void OnDestroy()
        {
            pickDataRef.Dispose();
        }
    }
}

