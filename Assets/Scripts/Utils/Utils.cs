using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityEditor;

namespace Utils
{

    public static class Prob
    {

        // binomialInterval creates a Vector2 containing the lower [0] and upper [1] confidence intervals
        // using a normal approximation
        public static Vector2 binomialInterval(int success, int trials)
        {
            // Normal approximation for 95%
            float zValue = 1.96f;  // Find library to calcullate zvalue based on confidence interval
            float p = success / (float) trials;
            float delta = zValue * Mathf.Sqrt(p * (1 - p) / trials);
            Vector2 CI = new Vector2(p - delta, p + delta);
            return CI;
        }


    }

    public static class ListExt
    {
        public static T PopAt<T>(this List<T> list, int index)
        {
            var r = list[index];
            list.RemoveAt(index);
            return r;
        }
        public static T PopFirst<T>(this List<T> list)
        {
            var r = list[0];
            list.RemoveAt(0);
            return r;
        }
        public static T PopLast<T>(this List<T> list)
        {

            if (list.Count > 0)
            {
                var r = list[list.Count - 1];
                list.RemoveAt(list.Count - 1);
                return r;
            }
            else
            {
                return default(T);
            }
                

        }
        public static T PopFirst<T>(this List<T> list, Predicate<T> predicate)
        {
            var index = list.FindIndex(predicate);
            var r = list[index];
            list.RemoveAt(index);
            return r;
        }

        public static T PopFirstOrDefault<T>(this List<T> list, Predicate<T> predicate) where T : class
        {
            var index = list.FindIndex(predicate);
            if (index > -1)
            {
                var r = list[index];
                list.RemoveAt(index);
                return r;
            }
            return null;
        }
    }

    public static class Drawer
    {
        // Visualizer functions
        public static void drawSpheres(List<Vector3> list, Color color, float radius)
        {
            Gizmos.color = color;
            for (int i = 0; i < list.Count; i++)
            {
                Gizmos.DrawSphere(list[i], radius);
            }
        }
        public static void drawSpheres(List<Node> list, Color color, float radius)
        {
            Gizmos.color = color;
            for (int i = 0; i < list.Count; i++)
            {
                Gizmos.DrawSphere(list[i].q, radius);
            }
        }
        public static void drawCircle(Vector3 origin, float radius, Color color)
        {
            Handles.color = color;
            Handles.DrawWireDisc(origin, Vector3.up, radius);
        }
        public static void debugDrawCircle(Vector3 origin, float radius, Color color, int segments)
        {
            float angle = 0f;
            Vector3 lastPoint = new Vector3(0, 0, 0);
            Vector3 thisPoint = new Vector3(0, 0, 0);

            for (int i = 0; i <= segments; i++)
            {
                thisPoint.x = radius * Mathf.Cos(angle) + origin.x;
                thisPoint.y = origin.y;
                thisPoint.z = radius * Mathf.Sin(angle) + origin.z;
                if (i > 0)
                {
                    Debug.DrawLine(lastPoint, thisPoint, color);
                }
                angle += 2 * Mathf.PI / segments;
                lastPoint = thisPoint;
            }
        }
    }


    public class ExperimentManager
    {

        public int maxExperimentRuns { get; set; }

        public int maxIterationExperiment { get; set; }
        public int iterationIncrement { get; set; }
        public int iterationExperiment { get; set; }

        public float gain { get; set; }
        public int experiment { get; set; }

        public String scenename { get; set; }

        

    }


    public class ResultsManager
    {
        public long planTime { get; set; }
        public long arrivalTime { get; set; }
        public float plannedCost { get; set; }
        public bool success { get; set; }
        public int nodes { get; set; }
        public int attempts { get; set; }
        public float executedCost { get; set; }
        public bool collision { get; set; }
    }
}
