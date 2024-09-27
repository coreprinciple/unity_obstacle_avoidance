using UnityEngine;
using System.Collections.Generic;

namespace avoidance.mono
{
    public class Agent : MonoBehaviour
    {
        private const float _RAY_SPHERE_RADIUS = 3.0f;

        [SerializeField] private Transform _cachedTransform;
        [SerializeField] private AvoidanceCommon _avoidanceData;

        private readonly List<Transform> _obstacles = new List<Transform>(100);

        private int _obstacleLayer;

        private Vector3 _lookDirection;
        private Vector3 _velocity;
        private Vector3 _targetPos;
        private bool _arrival;

        void Start()
        {
            _obstacleLayer = LayerMask.NameToLayer("obstacle");
            _lookDirection = _cachedTransform.forward;
            _arrival = true;
        }

        private Vector3 CollisionAvoidance(ref Transform threatening, Vector3 position, Vector3 velocity)
        {
            AgentUtil.CheckNearObstacles(_obstacles, _cachedTransform, _RAY_SPHERE_RADIUS, _obstacleLayer);

            Vector3 seeAhead = velocity.normalized * _avoidanceData.maxSeeAhead;
            Vector3 ahead = position + seeAhead;
            Vector3 ahead2 = position + seeAhead * 0.5f;

            Transform mostThreatening = AgentUtil.FindMostThreateningObstacle(_cachedTransform, position, ahead, ahead2, _obstacles, _avoidanceData.radius);
            Vector3 avoidance = Vector3.zero;

            if (mostThreatening != null)
            {
                Vector3 obstaclePos = mostThreatening.transform.position;
                obstaclePos.y = position.y;

                float dist = Vector3.Distance(position, obstaclePos);
                float ratio = (AvoidanceCommon.AVOID_DIST - Mathf.Min(dist, AvoidanceCommon.AVOID_DIST)) / AvoidanceCommon.AVOID_DIST;

                avoidance.x = ahead.x - mostThreatening.transform.position.x;
                avoidance.z = ahead.z - mostThreatening.transform.position.z;
                avoidance = avoidance.normalized * _avoidanceData.maxAvoidForce * ratio;
            }
            threatening = mostThreatening;
            return avoidance;
        }

        private Vector3 CollisionRaycast(Vector3 position, Quaternion rotation, Vector3 direction)
        {
            Vector3 deltaPos = Vector3.zero;

            for (int i = 0; i < AvoidanceCommon.NUM_RAYS; ++i)
            {
                Quaternion rot = rotation;
                Quaternion rotAround = Quaternion.AngleAxis((i / ((float)AvoidanceCommon.NUM_RAYS - 1)) * AvoidanceCommon.RAY_ANGLE * 2 - AvoidanceCommon.RAY_ANGLE, _cachedTransform.up);
                Vector3 dir = rot * rotAround * direction;
                Ray ray = new Ray(position, dir);

                if (Physics.Raycast(ray, out RaycastHit hitInfo, AvoidanceCommon.RAY_RANGE))
                    deltaPos -= (1.0f / AvoidanceCommon.NUM_RAYS) * _avoidanceData.maxVelocity * dir;
                else
                    deltaPos += (1.0f / AvoidanceCommon.NUM_RAYS) * _avoidanceData.maxVelocity * dir;
            }
            return deltaPos;
        }

        private Vector3 Seek(Vector3 position, Vector3 targetPos, float deltaTime)
        {
            Vector3 desiredVelocity = Vector3.Normalize(targetPos - position) * _avoidanceData.maxVelocity;
            return Vector3.Normalize(desiredVelocity - _velocity) * _avoidanceData.maxForce;
        }

        private void UpdateMove(float deltaTime)
        {
            Vector3 position = _cachedTransform.position;
            Vector3 targetPos = _targetPos;
            targetPos.y = position.y;

            float currentMain = Vector3.Distance(position, targetPos);
            Transform threatening = null;

            Vector3 steering = Vector3.zero;
            steering += Seek(position, targetPos, deltaTime);
            steering += CollisionAvoidance(ref threatening, position, _velocity);

            steering = steering.normalized * _avoidanceData.maxForce;
            steering = steering / _avoidanceData.mass;
            steering.y = 0.0f;

            _velocity = Vector3.Normalize(_velocity + steering) * _avoidanceData.maxSpeed * deltaTime;
            position = position + _velocity;

            Vector3 normV = _velocity.normalized;
            Quaternion rotation = Quaternion.Euler(normV);
            position += CollisionRaycast(position, rotation, normV) * deltaTime;
            _lookDirection = Vector3.Normalize(position - _cachedTransform.position);

            float moveDist = Vector3.Distance(_cachedTransform.position, position);
            float nextRemain = Vector3.Distance(position, targetPos);
            bool shouldMove = nextRemain > _avoidanceData.radius * 2.0f && moveDist < currentMain;
            bool overlap = threatening != null && Vector3.Distance(threatening.position, position) < _avoidanceData.radius;

            if (shouldMove || overlap)
                _cachedTransform.position = position;
            else
                _arrival = true;

            _cachedTransform.position = position;
        }

        private void UpdateRoatation(float deltaTime)
        {
            _cachedTransform.forward = Vector3.Lerp(_cachedTransform.forward, _lookDirection, deltaTime * _avoidanceData.rotLerpSpeed);
        }

        private void UpdateInput()
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);

            if (Physics.Raycast(ray, out RaycastHit hit, float.MaxValue))
            {
                _velocity = Vector3.zero;
                _targetPos = hit.point;
                _arrival = false;
            }
        }

        void Update()
        {
            if (Input.GetMouseButtonDown(0))
                UpdateInput();

            if (_arrival == false)
            {
                UpdateMove(Time.deltaTime);
                UpdateRoatation(Time.deltaTime);
            }
        }

        private void OnDrawGizmos()
        {
            Gizmos.DrawSphere(_cachedTransform.position, _RAY_SPHERE_RADIUS);

            for (int i = 0; i < AvoidanceCommon.NUM_RAYS; ++i)
            {
                Quaternion rot = _cachedTransform.rotation;
                Quaternion rotMode = Quaternion.AngleAxis((i / ((float)AvoidanceCommon.NUM_RAYS - 1)) * AvoidanceCommon.RAY_ANGLE * 2 - AvoidanceCommon.RAY_ANGLE, _cachedTransform.up);
                Vector3 direction = rot * rotMode * Vector3.forward;
                Gizmos.DrawRay(_cachedTransform.position, direction);
            }
        }
    }
}

