#define _FIX_SEED_
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Runtime.InteropServices;
using System.Threading;
using System.IO;
using Utils;
using CsvHelper;
using System.Globalization;
using UnityEditor;
using UnityEngine.SceneManagement;
public class BallControl : MonoBehaviour
{
    public bool FlagUseColor = false;
    public bool FlagMoveAgent = true;
    public bool FlagRewireTree = true;
    public bool FlagInformedSampling = true;
    public bool FlagRewiringCircle = true;

    static public bool flag_debug_on = false;

    public class regionInfo
    {
        //        public int numberOfDoneUpdates = -1;

        public float max_x, min_x;
        public float max_y, min_y;
        public float average_x, average_y;
        public ArrayList nodes_inside_region;// arraylist of indices

        public ArrayList listOfObstacles;

        public Vector2 cUp, cDown, cLeft, cRight;
        public Vector2 cUp_right, cUp_left, cDown_right, cDown_left;

        public Vector2 c_location;
        public regionInfo(Vector2 starting_pos, float size, Vector2 max_region, Vector2 in_location)
        {
            nodes_inside_region = new ArrayList();
            listOfObstacles = new ArrayList();

            cUp = new Vector2(float.MaxValue, float.MaxValue);
            cDown = new Vector2(float.MaxValue, float.MaxValue);
            cLeft = new Vector2(float.MaxValue, float.MaxValue);
            cRight = new Vector2(float.MaxValue, float.MaxValue);

            cUp_right = new Vector2(float.MaxValue, float.MaxValue);
            cUp_left = new Vector2(float.MaxValue, float.MaxValue);
            cDown_right = new Vector2(float.MaxValue, float.MaxValue);
            cDown_left = new Vector2(float.MaxValue, float.MaxValue);

            min_x = starting_pos[0];
            max_x = starting_pos[0] + size;
            if (max_x > max_region[0])
                max_x = max_region[0];
            min_y = starting_pos[1];
            max_y = starting_pos[1] + size;
            if (max_y > max_region[1])
                max_y = max_region[1];
            average_x = (max_x + min_x) / 2;
            average_y = (max_y + min_y) / 2;

            c_location = in_location;
        }
    }

    public class regionManager
    {
        public ArrayList mRegions;

        //        public ArrayList last_index_objects;

        //        int r_matrix = 0;// return regions around the current position of the agent with radius of r_matrix

        public regionManager(Vector2 p1, Vector2 p2, float size)//gets two point and devide the rectangle between the points into regions with the given size
        {
            //            last_index_objects = new ArrayList();

            float cur_x = p1[0];
            float cur_y = p1[1];
            mRegions = new ArrayList();
            while (cur_y < p2[1])
            {
                mRegions.Add(new ArrayList());
                while (cur_x < p2[0])
                {
                    ArrayList row_regions_i = (ArrayList)mRegions[mRegions.Count - 1];
                    (row_regions_i).Add(new regionInfo(new Vector2(cur_x, cur_y), size, p2, new Vector2(mRegions.Count - 1, row_regions_i.Count)));
                    cur_x += size;
                }
                cur_x = p1[0];
                cur_y += size;
            }
        }

        public void addNodeToRegion(mNodeRRT nNode)
        {
            //add node to the closest region
            Vector2 nNode_pos_on2D = get2D_on_theGround(nNode.objectPosition);
            Vector2 min_index = find_closest_region(nNode_pos_on2D);
            nNode.node_region_index = min_index;
            regionInfo closest_region = getRegionFromIndex(min_index);
            closest_region.nodes_inside_region.Add(nNode);
            //update other regions connections
            for (int i = 0; i < mRegions.Count; i++)
            {
                ArrayList columns_in_row = (ArrayList)mRegions[i];
                for (int j = 0; j < columns_in_row.Count; j++)
                {
                    regionInfo region_ij = (regionInfo)columns_in_row[j];
                    update_region_connections(region_ij, new Vector2(i, j), closest_region, min_index);
                }
            }
        }

        public int find_closest_node_inTree(Vector2 iPos)
        {
            Vector2 closest_index_region = find_closest_region(iPos);
            regionInfo closest_region = getRegionFromIndex(closest_index_region);

            float min_dis = float.MaxValue;
            int index_closest_node = -1;
            if (closest_region != null)
            {
                for (int r = 0; r < 9; r++)//we only have 9 regions, including itself
                {
                    regionInfo check_region = getNeighborsRegion(closest_region, r);
                    if (check_region != null)
                    {
                        for (int i = 0; i < check_region.nodes_inside_region.Count; i++)
                        {
                            mNodeRRT mNode = (mNodeRRT)check_region.nodes_inside_region[i];
                            float dis = (get2D_on_theGround(mNode.objectPosition) - iPos).magnitude;
                            if (dis < min_dis)
                            {
                                min_dis = dis;
                                index_closest_node = mNode.node_index_inTree;
                            }
                        }
                    }
                }
            }
            return index_closest_node;
        }

        public int find_Neighbors_inSearchBall_inTree(Vector3 Znew, float search_ball, ArrayList min_index_neighbor, ref int k_near_counter)
        {
            Vector2 closest_index_region = find_closest_region(get2D_on_theGround(Znew));
            regionInfo closest_region = getRegionFromIndex(closest_index_region);

            float min_dis = float.MaxValue;
            int min_index = -1;
            if (closest_region != null)
            {
                for (int r = 0; r < 9; r++)//we only have 9 regions, including itself
                {
                    regionInfo check_region = getNeighborsRegion(closest_region, r);
                    if (check_region != null)
                    {
                        for (int i = 0; i < check_region.nodes_inside_region.Count; i++)
                        {
                            mNodeRRT mNode = (mNodeRRT)check_region.nodes_inside_region[i];
                            float dis = (Znew - mNode.objectPosition).magnitude;
                            if (dis < search_ball)
                            {
                                min_index_neighbor.Add(mNode.node_index_inTree);
                                k_near_counter++;
                            }
                            if (dis < min_dis)
                            {
                                min_dis = dis;
                                min_index = mNode.node_index_inTree;
                            }
                        }
                    }
                }
            }
            return min_index;
        }

        public void update_neighbors_inSearchBall_inTree(mNodeRRT nParentNode, float search_ball, ArrayList min_index_neighbor)
        {
            Vector2 closest_index_region = find_closest_region(get2D_on_theGround(nParentNode.objectPosition));
            regionInfo closest_region = getRegionFromIndex(closest_index_region);

            if (closest_region != null)
            {
                for (int r = 0; r < 9; r++)//we only have 9 regions, including itself
                {
                    regionInfo check_region = getNeighborsRegion(closest_region, r);
                    if (check_region != null)
                    {
                        for (int i = 0; i < check_region.nodes_inside_region.Count; i++)//TODO:change this part
                        {
                            mNodeRRT iNode = (mNodeRRT)check_region.nodes_inside_region[i];
                            float dis = (nParentNode.objectPosition - iNode.objectPosition).magnitude;
                            if (dis < search_ball)
                            {
                                int index_node_inTree = iNode.node_index_inTree;
                                if (index_node_inTree != nParentNode.node_index_inTree && !min_index_neighbor.Contains(index_node_inTree))
                                {
                                    min_index_neighbor.Add(index_node_inTree);
                                }
                            }
                        }
                    }
                }
            }

            return;
        }

        public int update_cost_to_reach_for_dynamicObstacles(ArrayList TreeNodes, ArrayList obstacles_pos, ArrayList last_obstacle_pos, int update_needed_obstacles, int current_root
            , float m_collide_region, ref bool update_path_toGoal, int goal_index_inTree, int numberOfUpdatesForCost, ArrayList index_obstacles, ref float min_blocked_node_cost)
        {

            bool check_tree_for_update = false;

            int row_region = mRegions.Count;
            int col_region = 0;
            if (row_region > 0)
                col_region = ((ArrayList)mRegions[0]).Count;

            for (int i = 0; i < row_region; i++)
            {
                for (int j = 0; j < col_region; j++)
                {
                    regionInfo region_ij = getRegionFromIndex(new Vector2(i, j));
                    region_ij.listOfObstacles.Clear();
                }
            }

            ArrayList closest_region_to_obstacles = new ArrayList(row_region * col_region);
            ArrayList closest_region_to_i_obstacles = new ArrayList(row_region * col_region);
            for (int i = 0; i < row_region * col_region; i++)
            {
                closest_region_to_obstacles.Add(new ArrayList(obstacles_pos.Count));
                closest_region_to_i_obstacles.Add(new ArrayList(obstacles_pos.Count));
            }
            for (int i = 0; i < obstacles_pos.Count; i++)
            {
                int index_obstacle_i = (int)index_obstacles[i];

                Vector3 obs_pos_i = (Vector3)obstacles_pos[i];
                //                Vector3 l_obs_pos_i = (Vector3)last_obstacle_pos[i];

                Vector2 closest_index_region = find_closest_region(get2D_on_theGround(obs_pos_i));

                int index_in_closest_region_array = (int)(closest_index_region[0] * col_region + closest_index_region[1]);
                if (index_in_closest_region_array >= 0 && index_in_closest_region_array < closest_region_to_obstacles.Count)
                {
                    ((ArrayList)closest_region_to_obstacles[index_in_closest_region_array]).Add(obs_pos_i);
                    ((ArrayList)closest_region_to_i_obstacles[index_in_closest_region_array]).Add(index_obstacle_i);
                }
            }

            ArrayList check_regions_numbers = new ArrayList();
            ArrayList check_obstacles_for_region = new ArrayList();
            for (int o = 0; o < closest_region_to_obstacles.Count; o++)
            {
                ArrayList list_obstacles_inClosest_region = (ArrayList)closest_region_to_obstacles[o];
                ArrayList list_Iobstacles_inClosest_region = (ArrayList)closest_region_to_i_obstacles[o];

                if (list_obstacles_inClosest_region.Count > 0)
                {
                    int r_region = (int)(o / col_region);
                    int c_region = o % col_region;
                    regionInfo closest_region = getRegionFromIndex(new Vector2(r_region, c_region));

                    if (closest_region != null)
                    {
                        closest_region.listOfObstacles = list_Iobstacles_inClosest_region;//update its dynamic obstacles
                        for (int r = 0; r < 9; r++)//we only have 9 regions, including itself
                        {
                            regionInfo check_region = getNeighborsRegion(closest_region, r);
                            if (check_region != null)
                            {
                                int check_region_num = (int)(check_region.c_location[0]) * col_region + (int)check_region.c_location[1];
                                if (!check_regions_numbers.Contains(check_region_num))
                                {
                                    check_regions_numbers.Add(check_region_num);
                                    check_obstacles_for_region.Add(list_obstacles_inClosest_region);
                                }
                                else
                                {
                                    int i_check_region = check_regions_numbers.IndexOf(check_region_num);
                                    for (int m_o = 0; m_o < list_obstacles_inClosest_region.Count; m_o++)
                                    {
                                        ((ArrayList)check_obstacles_for_region[i_check_region]).Add(list_obstacles_inClosest_region[m_o]);
                                    }
                                }
                            }
                        }
                    }
                }
            }

            min_blocked_node_cost = float.MaxValue;

            for (int r = 0; r < check_regions_numbers.Count; r++)
            {
                int o = (int)check_regions_numbers[r];
                int r_region = (int)(o / col_region);
                int c_region = o % col_region;
                regionInfo check_region = getRegionFromIndex(new Vector2(r_region, c_region));
                ArrayList list_obstacles_inClosest_region = (ArrayList)check_obstacles_for_region[r];
                for (int i = 0; i < check_region.nodes_inside_region.Count; i++)
                {
                    mNodeRRT iNode = (mNodeRRT)check_region.nodes_inside_region[i];
                    bool update_local_str = check_node_obstacles(TreeNodes, iNode, list_obstacles_inClosest_region, current_root, m_collide_region, goal_index_inTree
                        , ref update_path_toGoal, numberOfUpdatesForCost, ref min_blocked_node_cost);
                    if (!check_tree_for_update)
                        check_tree_for_update = update_local_str;
                }
            }
            if (check_tree_for_update)
            {
                //                last_index_objects = index_obstacles;
                return update_needed_obstacles + 1;
            }
            return update_needed_obstacles;
        }

        public void find_closest_nodes_ToGoals(ArrayList goal_points, ArrayList arr_closest_index, ArrayList arr_min_dis)
        {
            int row_region = mRegions.Count;
            int col_region = 0;
            if (row_region > 0)
                col_region = ((ArrayList)mRegions[0]).Count;
            ArrayList closest_region_to_goals = new ArrayList(row_region * col_region);
            for (int i = 0; i < row_region * col_region; i++)
            {
                closest_region_to_goals.Add(new ArrayList(goal_points.Count));
            }
            for (int i = 0; i < goal_points.Count; i++)
            {
                arr_closest_index.Add(-1);
                arr_min_dis.Add(float.MaxValue);

                Vector3 goal_pos_i = (Vector3)goal_points[i];
                Vector2 closest_index_region = find_closest_region(get2D_on_theGround(goal_pos_i));
                ((ArrayList)closest_region_to_goals[(int)(closest_index_region[0] * col_region + closest_index_region[1])]).Add(i);
            }

            for (int o = 0; o < closest_region_to_goals.Count; o++)//moving on the regions to see which regions have goal_points inside them
            {
                ArrayList list_index_goals_inClosest_region = (ArrayList)closest_region_to_goals[o];
                if (list_index_goals_inClosest_region.Count > 0)
                {
                    int r_region = (int)(o / col_region);
                    int c_region = o % col_region;
                    regionInfo closest_region = getRegionFromIndex(new Vector2(r_region, c_region));
                    if (closest_region != null)
                    {
                        for (int r = 0; r < 9; r++)//we only have 9 regions, including itself
                        {
                            regionInfo check_region = getNeighborsRegion(closest_region, r);
                            if (check_region != null)
                            {
                                for (int i = 0; i < check_region.nodes_inside_region.Count; i++)//index_node in region
                                {
                                    mNodeRRT iNode = (mNodeRRT)check_region.nodes_inside_region[i];
                                    for (int g = 0; g < list_index_goals_inClosest_region.Count; g++)
                                    {
                                        int index_goal_inGoalPoints = (int)list_index_goals_inClosest_region[g];
                                        Vector3 c_goal_point = (Vector3)goal_points[index_goal_inGoalPoints];
                                        float c_dis = (c_goal_point - iNode.objectPosition).magnitude;
                                        if (c_dis < (float)arr_min_dis[index_goal_inGoalPoints])
                                        {
                                            arr_min_dis[index_goal_inGoalPoints] = c_dis;
                                            arr_closest_index[index_goal_inGoalPoints] = iNode.node_index_inTree;
                                        }//if
                                    }//for goals in list region
                                }//for nodes in near regions
                            }//if existence of closest region
                        }//for on near regions
                    }//if closest region exists
                }//if any goal exists in the region
            }
            return;
        }

        //public ArrayList return_nodes_inside_closest_region(Vector3 cur_pos, int numberOfUpdates)
        //{
        //    int row_region = mRegions.Count;
        //    int col_region = 0;
        //    if (row_region > 0)
        //        col_region = ((ArrayList)mRegions[0]).Count;

        //    Vector2 closest_index_region = find_closest_region(get2D_on_theGround(cur_pos));
        //    regionInfo closest_region = getRegionFromIndex(closest_index_region);

        //    bool flag_return_nodes = false;
        //    ArrayList ret_nodes = closest_region.nodes_inside_region;

        //    if (closest_region.numberOfDoneUpdates < numberOfUpdates)
        //    {
        //        flag_return_nodes = true;
        //        closest_region.numberOfDoneUpdates = numberOfUpdates;
        //        r_matrix = 0;
        //    }

        //    bool out_top = false;
        //    bool out_down = false;
        //    bool out_left = false;
        //    bool out_right = false;

        //    int s_r = (int)closest_index_region[0] - r_matrix;
        //    if (s_r < 0)
        //    {
        //        s_r = 0;
        //        out_down = true;
        //    }
        //    int s_c = (int)closest_index_region[1] - r_matrix;
        //    if (s_c < 0)
        //    {
        //        s_c = 0;
        //        out_left = true;
        //    }
        //    int e_r = (int)closest_index_region[0] + r_matrix;
        //    if (e_r >= row_region)
        //    {
        //        e_r = row_region - 1;
        //        out_top = true;
        //    }
        //    int e_c = (int)closest_index_region[1] + r_matrix;
        //    if (e_c >= col_region)
        //    {
        //        e_c = col_region - 1;
        //        out_right = true;
        //    }
        //    if (out_down && out_left && out_right && out_top)
        //    {
        //        flag_return_nodes = true;
        //        closest_region.numberOfDoneUpdates = numberOfUpdates;
        //        r_matrix = 0;
        //    }

        //    for (int i = s_r; i <= e_r && !flag_return_nodes; i++)
        //    {
        //        Vector2 choose_index_region = new Vector2(i, s_c);
        //        regionInfo choose_region = getRegionFromIndex(choose_index_region);
        //        if (choose_region.numberOfDoneUpdates < numberOfUpdates)
        //        {
        //            flag_return_nodes = true;
        //            choose_region.numberOfDoneUpdates = numberOfUpdates;
        //            ret_nodes = choose_region.nodes_inside_region;
        //        }
        //    }
        //    for (int i = s_r; i <= e_r && !flag_return_nodes; i++)
        //    {
        //        Vector2 choose_index_region = new Vector2(i, e_c);
        //        regionInfo choose_region = getRegionFromIndex(choose_index_region);
        //        if (choose_region.numberOfDoneUpdates < numberOfUpdates)
        //        {
        //            flag_return_nodes = true;
        //            choose_region.numberOfDoneUpdates = numberOfUpdates;
        //            ret_nodes = choose_region.nodes_inside_region;
        //        }
        //    }
        //    for (int i = s_c + 1; i < e_c && !flag_return_nodes; i++)
        //    {
        //        Vector2 choose_index_region = new Vector2(s_r, i);
        //        regionInfo choose_region = getRegionFromIndex(choose_index_region);
        //        if (choose_region.numberOfDoneUpdates < numberOfUpdates)
        //        {
        //            flag_return_nodes = true;
        //            choose_region.numberOfDoneUpdates = numberOfUpdates;
        //            ret_nodes = choose_region.nodes_inside_region;
        //        }
        //    }
        //    for (int i = s_c + 1; i < e_c && !flag_return_nodes; i++)
        //    {
        //        Vector2 choose_index_region = new Vector2(e_r, i);
        //        regionInfo choose_region = getRegionFromIndex(choose_index_region);
        //        if (choose_region.numberOfDoneUpdates < numberOfUpdates)
        //        {
        //            flag_return_nodes = true;
        //            choose_region.numberOfDoneUpdates = numberOfUpdates;
        //            ret_nodes = choose_region.nodes_inside_region;
        //        }
        //    }
        //    if (!flag_return_nodes)
        //        r_matrix++;
        //    return ret_nodes;
        //}

        public void updateReachableNode(mNodeRRT cNode, Vector3 cur_pos_agent, float radius_check_obstacle_region, float radius_contact_region, ArrayList c_pos_obstacles)
        {
            if (!cNode.reachable_node)
            {
                if ((cNode.objectPosition - cur_pos_agent).magnitude > radius_check_obstacle_region + 2 * radius_contact_region)
                    cNode.reachable_node = true;
                else
                {
                    regionInfo l_node_region = getRegionFromIndex(cNode.node_region_index);
                    ArrayList c_obstacles = new ArrayList();

                    for (int r = 0; r < 9; r++)// TODO: if the contact region is bigger so this should change
                    {
                        regionInfo nr = getNeighborsRegion(l_node_region, r);
                        if (nr != null)
                        {
                            for (int o = 0; o < nr.listOfObstacles.Count; o++)
                            {
                                c_obstacles.Add(nr.listOfObstacles[o]);
                            }
                        }
                    }

                    bool flag_isHit = false;
                    for (int i = 0; i < c_obstacles.Count && !flag_isHit; i++)
                    {
                        int i_obstacle_i = (int)c_obstacles[i];
                        Vector3 obstacle_posi = (Vector3)c_pos_obstacles[i_obstacle_i];
                        if ((cNode.objectPosition - obstacle_posi).magnitude < radius_contact_region)
                            flag_isHit = true;
                    }
                    if (!flag_isHit)
                        cNode.reachable_node = true;
                }
            }
            return;
        }

        private bool check_node_obstacles(ArrayList TreeNodes, mNodeRRT iNode, ArrayList obstacles_pos, int current_root, float m_collide_region
                                        , int goal_index_inTree, ref bool update_path_toGoal, int numberOfUpdatesForCost, ref float min_blocked_node_cost)
        {
            float not_reaching_ToGoal_value = float.MaxValue;
            bool flag_update_structure = false;
            if (iNode.node_index_inTree != current_root)
            {
                bool hit_obstacle = false;
                for (int j = 0; j < obstacles_pos.Count; j++)
                {
                    Vector3 obstacle_pos_j = (Vector3)obstacles_pos[j];
                    if ((obstacle_pos_j - iNode.objectPosition).magnitude < m_collide_region)
                    {
                        if (iNode.cost_to_reach != not_reaching_ToGoal_value)
                        {
                            flag_update_structure = true;
                            if (min_blocked_node_cost > iNode.cost_to_reach)
                                min_blocked_node_cost = iNode.cost_to_reach;
                        }
                        iNode.reachable_node = false;
                        iNode.cost_to_reach = not_reaching_ToGoal_value;
                        hit_obstacle = true;
                    }
                }

                // must return last situation that node had, so first update its cost_to_reach the
                bool reachable_value_nodei = iNode.reachable_node;
                int f_index_i = iNode.indexOfFather;
                if (!hit_obstacle)
                {
                    iNode.reachable_node = true;
                    if (iNode.getCostToReach(TreeNodes, numberOfUpdatesForCost) == not_reaching_ToGoal_value)
                    {
                        //flag_update_structure = true;
                        if (f_index_i != -1)
                        {
                            mNodeRRT f_node = (mNodeRRT)TreeNodes[f_index_i];
                            iNode.cost_to_reach = f_node.getCostToReach(TreeNodes, numberOfUpdatesForCost) + (iNode.objectPosition - f_node.objectPosition).magnitude;
                        }
                        else
                            iNode.cost_to_reach = 0;
                    }
                    if (!reachable_value_nodei && goal_index_inTree != -1)
                    {
                        update_path_toGoal = true;
                    }
                }
            }
            return flag_update_structure;
        }

        private regionInfo getNeighborsRegion(regionInfo cRegion, int neighbor_i)
        {
            switch (neighbor_i)
            {
                case 0:
                    return cRegion;
                case 1:
                    return getRegionFromIndex(cRegion.cLeft);
                case 2:
                    return getRegionFromIndex(cRegion.cUp_left);
                case 3:
                    return getRegionFromIndex(cRegion.cUp);
                case 4:
                    return getRegionFromIndex(cRegion.cUp_right);
                case 5:
                    return getRegionFromIndex(cRegion.cRight);
                case 6:
                    return getRegionFromIndex(cRegion.cDown_right);
                case 7:
                    return getRegionFromIndex(cRegion.cDown);
                case 8:
                    return getRegionFromIndex(cRegion.cDown_left);
                default:
                    return cRegion;
            }
        }

        private regionInfo getRegionFromIndex(Vector2 index_region)
        {
            if (index_region[0] != float.MaxValue && index_region[1] != float.MaxValue)
            {
                if (index_region[0] < 0 || index_region[0] >= mRegions.Count)
                {
                    return null;
                }
                ArrayList columns_in_row = (ArrayList)mRegions[(int)index_region[0]];
                if (index_region[1] < 0 || index_region[1] >= columns_in_row.Count)
                {
                    return null;
                }
                return (regionInfo)columns_in_row[(int)index_region[1]];
            }
            return null;
        }

        private void update_region_connections(regionInfo iRegion, Vector2 index_iRegion, regionInfo updated_region, Vector2 index_updated_region)
        {
            //iRegion.cUp
            if (index_iRegion[1] == index_updated_region[1])//if they have same columns
            {
                if (index_updated_region[0] > index_iRegion[0])//if the updated region is located on the top of the iRegion
                {
                    float cur_dis = (iRegion.cUp - index_iRegion).magnitude;
                    float n_dis = (index_updated_region - index_iRegion).magnitude;
                    if (n_dis < cur_dis)
                    {
                        iRegion.cUp = index_updated_region;

                        if (iRegion.nodes_inside_region.Count > 0)
                        {
                            updated_region.cDown = index_iRegion;
                        }
                    }
                }
            }
            //iRegion.cDown
            if (index_iRegion[1] == index_updated_region[1])//if they have same columns
            {
                if (index_updated_region[0] < index_iRegion[0])//if the updated region is located on the downside of the iRegion
                {
                    float cur_dis = (iRegion.cDown - index_iRegion).magnitude;
                    float n_dis = (index_updated_region - index_iRegion).magnitude;
                    if (n_dis < cur_dis)
                    {
                        iRegion.cDown = index_updated_region;

                        if (iRegion.nodes_inside_region.Count > 0)
                        {
                            updated_region.cUp = index_iRegion;
                        }
                    }
                }
            }
            //iRegion.cLeft
            if (index_iRegion[0] == index_updated_region[0])//if they have same rows
            {
                if (index_updated_region[1] < index_iRegion[1])//if the updated region is located on the leftside of the iRegion
                {
                    float cur_dis = (iRegion.cLeft - index_iRegion).magnitude;
                    float n_dis = (index_updated_region - index_iRegion).magnitude;
                    if (n_dis < cur_dis)
                    {
                        iRegion.cLeft = index_updated_region;

                        if (iRegion.nodes_inside_region.Count > 0)
                        {
                            updated_region.cRight = index_iRegion;
                        }
                    }
                }
            }
            //iRegion.cRight
            if (index_iRegion[0] == index_updated_region[0])//if they have same rows
            {
                if (index_updated_region[1] > index_iRegion[1])//if the updated region is located on the rightside of the iRegion
                {
                    float cur_dis = (iRegion.cRight - index_iRegion).magnitude;
                    float n_dis = (index_updated_region - index_iRegion).magnitude;
                    if (n_dis < cur_dis)
                    {
                        iRegion.cRight = index_updated_region;

                        if (iRegion.nodes_inside_region.Count > 0)
                        {
                            updated_region.cLeft = index_iRegion;
                        }
                    }
                }
            }
            //iRegion.cUp_left
            if (index_iRegion[0] < index_updated_region[0])//up
            {
                if (index_iRegion[1] > index_updated_region[1])//left
                {
                    float cur_dis = (iRegion.cUp_left - index_iRegion).magnitude;
                    float n_dis = (index_updated_region - index_iRegion).magnitude;
                    if (n_dis < cur_dis)
                    {
                        iRegion.cUp_left = index_updated_region;

                        if (iRegion.nodes_inside_region.Count > 0)
                        {
                            updated_region.cDown_right = index_iRegion;
                        }
                    }
                }
            }
            //iRegion.cUp_right
            if (index_iRegion[0] < index_updated_region[0])//up
            {
                if (index_iRegion[1] < index_updated_region[1])//right
                {
                    float cur_dis = (iRegion.cUp_right - index_iRegion).magnitude;
                    float n_dis = (index_updated_region - index_iRegion).magnitude;
                    if (n_dis < cur_dis)
                    {
                        iRegion.cUp_right = index_updated_region;

                        if (iRegion.nodes_inside_region.Count > 0)
                        {
                            updated_region.cDown_left = index_iRegion;
                        }
                    }
                }
            }
            //iRegion.cDown_right
            if (index_iRegion[0] > index_updated_region[0])//down
            {
                if (index_iRegion[1] < index_updated_region[1])//right
                {
                    float cur_dis = (iRegion.cDown_right - index_iRegion).magnitude;
                    float n_dis = (index_updated_region - index_iRegion).magnitude;
                    if (n_dis < cur_dis)
                    {
                        iRegion.cDown_right = index_updated_region;

                        if (iRegion.nodes_inside_region.Count > 0)
                        {
                            updated_region.cUp_left = index_iRegion;
                        }
                    }
                }
            }
            //iRegion.cDown_left
            if (index_iRegion[0] > index_updated_region[0])//down
            {
                if (index_iRegion[1] > index_updated_region[1])//left
                {
                    float cur_dis = (iRegion.cDown_left - index_iRegion).magnitude;
                    float n_dis = (index_updated_region - index_iRegion).magnitude;
                    if (n_dis < cur_dis)
                    {
                        iRegion.cDown_left = index_updated_region;

                        if (iRegion.nodes_inside_region.Count > 0)
                        {
                            updated_region.cUp_right = index_iRegion;
                        }
                    }
                }
            }
            return;
        }

        private Vector2 find_closest_region(Vector2 pos)
        {
            float min_dis = float.MaxValue;
            int min_index_i = -1;//row
            int min_index_j = -1;//column
            for (int i = 0; i < mRegions.Count; i++)
            {
                ArrayList columns_in_row = (ArrayList)mRegions[i];
                for (int j = 0; j < columns_in_row.Count; j++)
                {
                    regionInfo region_ij = (regionInfo)columns_in_row[j];
                    float cur_dis = ((new Vector2(region_ij.average_x, region_ij.average_y)) - pos).magnitude;
                    if (cur_dis < min_dis)
                    {
                        min_index_i = i;
                        min_index_j = j;
                        min_dis = cur_dis;
                    }
                }
            }
            return new Vector2(min_index_i, min_index_j);
        }

        private Vector2 get2D_on_theGround(Vector3 node_pos)
        {
            return new Vector2(node_pos[0], node_pos[2]);
        }
    }

    public class mNodeRRT
    {
        //specification of the object
        public Vector3 objectPosition;//x,y(height),z
        public Vector2 objectDirection;//(x,y)
        public float shapeRadius;
        //Link to other nodes in the tree
        public ArrayList child_nodes_index;
        public int indexOfFather;
        public int node_index_inTree = -1;

        //huristic to reach goal and cost_to_reach current node from root
        public float node_value = 0;
        public float cost_to_reach = 0;

        //variables for updating cost_to_reach and node_value
        public int numberOfUpdates_cost = -1;//for updating cost_to_reach
        public int numberOfUpdates_value = -1;//for updating node_value
        public bool reachable_node = true; //for dynamic environment
        public int numberOfUpdate_structure = -1;//for updating structure

        //nodes inside the search ball
        public ArrayList nodes_inside_search_ball;

        public Vector2 node_region_index;

        public float getNodeWorth(ArrayList TreeNodes, int numberOfNeededUpdates_cost, int numberOfNeededUpdates_value, Vector3 goal_point_3d)
        {
            return getCostToReach(TreeNodes, numberOfNeededUpdates_cost) + getNode_Value(goal_point_3d, numberOfNeededUpdates_value);
        }

        public mNodeRRT()
        {
            objectPosition = new Vector3();
            objectDirection = new Vector2();
            child_nodes_index = new ArrayList();
            indexOfFather = -1;
            nodes_inside_search_ball = new ArrayList();
            node_region_index = new Vector2();
        }

        public mNodeRRT(float x, float y, float z, float dirX, float dirY)
        {
            objectPosition = new Vector3(x, y, z);
            objectDirection = new Vector2(dirX, dirY);
            child_nodes_index = new ArrayList();
            indexOfFather = -1;
            nodes_inside_search_ball = new ArrayList();
            node_region_index = new Vector2();
        }

        public mNodeRRT(float[] pos, float[] dir)//pos:(x,y,z);  dir:(x,y)
        {
            objectPosition = new Vector3(pos[0], pos[1], pos[2]);
            objectDirection = new Vector2(dir[0], dir[1]);
            child_nodes_index = new ArrayList();
            indexOfFather = -1;
            nodes_inside_search_ball = new ArrayList();
            node_region_index = new Vector2();
        }

        public mNodeRRT(Vector3 pos, Vector2 dir)//pos:(x,y,z);  dir:(x,y)
        {
            objectPosition = new Vector3(pos[0], pos[1], pos[2]);
            objectDirection = new Vector2(dir[0], dir[1]);
            child_nodes_index = new ArrayList();
            indexOfFather = -1;
            nodes_inside_search_ball = new ArrayList();
            node_region_index = new Vector2();
        }

        public float getCostToReach(ArrayList TreeNodes, int numberOfNeededUpdate)
        {
            bool flag_continue = true;
            ArrayList update_nodes_costs = new ArrayList();
            //find which nodes need to be updated
            mNodeRRT cNode = this;
            update_nodes_costs.Add(cNode);
            while (flag_continue)
            {
                if (cNode.indexOfFather != -1)
                {
                    mNodeRRT fNode = (mNodeRRT)TreeNodes[cNode.indexOfFather];
                    if (fNode.numberOfUpdates_cost < numberOfNeededUpdate)
                    {
                        update_nodes_costs.Add(fNode);
                        cNode = fNode;
                    }
                    else
                        flag_continue = false;
                }
                else
                    flag_continue = false;
            }
            //update the cost from last one added to the first one
            for (int i = update_nodes_costs.Count - 1; i >= 0; i--)
            {
                cNode = (mNodeRRT)update_nodes_costs[i];
                mNodeRRT fNode = null;
                if (i + 1 < update_nodes_costs.Count)
                {
                    fNode = (mNodeRRT)update_nodes_costs[i + 1];
                }
                else
                {
                    if (cNode.indexOfFather != -1)
                        fNode = (mNodeRRT)TreeNodes[cNode.indexOfFather];
                    else
                        cNode.cost_to_reach = 0;
                }
                if (fNode != null)
                {
                    if (cNode.cost_to_reach == float.MaxValue || fNode.cost_to_reach == float.MaxValue)
                    {
                        cNode.cost_to_reach = float.MaxValue;
                    }
                    else
                    {
                        cNode.cost_to_reach = fNode.cost_to_reach + (cNode.objectPosition - fNode.objectPosition).magnitude;
                    }
                }
                cNode.numberOfUpdates_cost = numberOfNeededUpdate;
            }
            return this.cost_to_reach;
        }

        public float getNode_Value(Vector3 goal_point_3d, int numberOfNeededUpdate)
        {
            if (numberOfUpdates_value < numberOfNeededUpdate)
            {
                node_value = update_node_value(goal_point_3d);
                numberOfUpdates_value = numberOfNeededUpdate;
            }
            return node_value;
        }

        public float update_node_value(Vector3 goal_point_3d)
        {
            node_value = (this.objectPosition - goal_point_3d).magnitude;
            return node_value;
        }
    }

    public class m_queue_manager
    {
        ArrayList nodes_in_queue;
        int c_counter_add_node = 0;

        public int getSize() { return c_counter_add_node; }

        public m_queue_manager()
        {
            c_counter_add_node = 0;
            nodes_in_queue = new ArrayList(10000);
        }

        public bool check_empty()
        {
            bool c_empty = false;
            if (nodes_in_queue.Count < 1 || c_counter_add_node == 0)
                c_empty = true;
            return c_empty;
        }

        public mNodeRRT getFirstNode()
        {
            if (nodes_in_queue == null)
                return null;
            if (nodes_in_queue.Count > 0 && c_counter_add_node > 0)
            {
                mNodeRRT c_node = (mNodeRRT)nodes_in_queue[0];
                nodes_in_queue.RemoveAt(0);
                c_counter_add_node--;
                return c_node;
            }
            return null;
        }

        public void insertNodeFirst(mNodeRRT iNode)
        {
            if (nodes_in_queue.Count == 0)
                nodes_in_queue.Add(iNode);
            else
                nodes_in_queue.Insert(0, iNode);
            c_counter_add_node++;
            return;
        }

        public void addNodeToQueue(mNodeRRT cNode)
        {
            if (c_counter_add_node >= 0 && c_counter_add_node <= nodes_in_queue.Count - 1)
            {
                nodes_in_queue[c_counter_add_node] = cNode;
            }
            else
            {
                nodes_in_queue.Add(cNode);
            }
            c_counter_add_node++;
            return;
        }

        public float addNodeToQueueForStructure(ArrayList TreeNodes, mNodeRRT cNode, int numberOfneeded_changeStructure, bool FlagUseColor)
        {
            float added_node_cost = -1;
            if (cNode.numberOfUpdate_structure < numberOfneeded_changeStructure)
            {
                addNodeToQueue(cNode);
                cNode.numberOfUpdate_structure = numberOfneeded_changeStructure;
                if (cNode.indexOfFather >= 0 && flag_debug_on && FlagUseColor)
                {
                    mNodeRRT fNode = (mNodeRRT)TreeNodes[cNode.indexOfFather];
                    //Debug.DrawLine(cNode.objectPosition, fNode.objectPosition, Color.magenta);
                }
                if (cNode.cost_to_reach != float.MaxValue)
                    added_node_cost = cNode.cost_to_reach;
            }
            return added_node_cost;
        }

        public void restart_queue(mNodeRRT cNode)
        {
            c_counter_add_node = 0;
            addNodeToQueue(cNode);
        }
    }

    public class LazyRRT
    {
        /// <summary>
        /// 
        /// </summary>

        ////////////////////////////// defination of variables////////////////////////////////
        //Simulation
        public int mSimulation_numberOfNodes = 2000;//number of nodes in the environment before moving to the goal
        public ArrayList cost_of_found_paths;//during the movement the path is going to change a couple of times, this will save their cost
        public int numberOfChangedInPath = 0;//during the movement the path is going to change a couple of times, this will tell how many times it changed
        public int iteration_needed_for_firstPath = 0;//iteration needed for returning the first path
        public float cost_of_taken_path = 0;//cost_of_taken_path

        //tree variables
        public ArrayList TreeNodes;//root is assumed variable
        int current_root = 0;
        public Vector2 goal_point;//assumed 2D: x,z; changable: multi-query tree
        private float rateTowardGoal = 0.1f;
        //int numberOfRRTSamples = ;
        int path_length_onTree = 0; // for dividing the path from one node to the another

        //real-time path planning
        ArrayList best_path_soFar;

        //informed RRT* var
        public float cur_cost_path = float.MaxValue;
        public float best_cost_path = float.MaxValue;
        float cost_path_inside_notReachArea = 0;
        bool flag_calculate_path_cost = false;

        //obstacle variables (keep current pos to sense the movement of the obstacles)
        float radius_check_obstacle_region = 10, radius_contact_region;// r_o and r_b
        ArrayList cur_pos_obstacles;
        Vector3 cur_pos_agent;

        //when it is necessary to change the cost_to_reach value of each node, is decided by the following variables
        int numberOfUpdatingRoot = 0;
        int numberOfUpdatingObstacles = 0;
        int numberOfUpdatingGoals = 0;
        int numberOfUpdatingStr = 0;

        //environment & agent variables
        float max_range_x = 15, min_range_x = -15;
        float max_range_z = 15, min_range_z = -15;
        float cRadius;

        //these values are for the node value not for cost to reach
        private float goal_value = -float.MaxValue;//cost to reach to goal 
        private float not_reaching_ToGoal_value = float.MaxValue;
        int max_k_step_forward = 100;

        //path variables
        private bool flag_find_path = false;
        ArrayList path_from_root;
        int goal_index_inTree = -1;

        //local sampling
        m_queue_manager queue_random_samples;
        m_queue_manager queue_onTree_nodes;

        //create regions
        regionManager mRegionManager;

        //handeling dynamic obstacles
        float c_distance_ring = 0;
        public bool FlagRewireTree = true;
        public bool FlagInformedSampling = true;
        public bool FlagRewiringCircle = true;
        public bool FlagUseColor = true;


        // added variables
        long start_ms;
        public int iterations;
        public int iterationsToFinish;
        public bool finishedGrowingTree_;
        public int maxIterations;
        public LayerMask fixedObstaclesLayer;
        ////////////////////////// end of defination of variables /////////////////////////////

        /// <param name="pos">initial pos of agent</param>
        /// <param name="dir">initial direction of agent</param>
        /// <param name="radius">agent is assumed to be a ball with the size of radius</param>
        /// <param name="initial_goal">initial goal point</param>
        public LazyRRT(Vector3 pos, Vector2 dir, float radius, Vector2 initial_goal, int maxIterations, float radius_check_obstacle_region, float radius_contact_region, Vector2 xBound, Vector2 zBound)
        {
            //added variables
            this.start_ms = DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;
            this.iterations = 0;
            this.maxIterations = maxIterations;
            this.radius_check_obstacle_region = radius_check_obstacle_region;
            this.radius_contact_region = radius_contact_region;//r_b
            this.fixedObstaclesLayer = LayerMask.GetMask("Obstacles");
            //environment & agent variables
            this.max_range_x = xBound[1];
            this.min_range_x = xBound[0];
            this.max_range_z = zBound[1];
            this.min_range_z = zBound[0];

            // real-time path palnning
            best_path_soFar = new ArrayList();

            //For simulation
            cost_of_found_paths = new ArrayList();

            //queue 
            queue_random_samples = new m_queue_manager();
            queue_onTree_nodes = new m_queue_manager();

            /////obstacles' poses
            cur_pos_obstacles = new ArrayList();
            cur_pos_agent = pos;

            /////region manager
            mRegionManager = new regionManager(new Vector2(min_range_x, min_range_z), new Vector2(max_range_x, max_range_z), 2f);//grid size 2*2

            goal_point = new Vector2();
            TreeNodes = new ArrayList(5 * ((int)(max_range_x - min_range_x)) * ((int)(max_range_z - min_range_z)));

            cRadius = radius;//r_s and r_g

            path_from_root = new ArrayList();

            set_goal_point(initial_goal[0], initial_goal[1]);
            add_node(pos, dir, radius, -1);//add first node
        }
        public long getPlanTime()
        {
            long cur_time_ms = DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;

            return (cur_time_ms - this.start_ms);
        }
        public bool isPathFound() { return flag_find_path; }

        public void restart_parent_node_values(int sIndex, Vector3 goal_pos_3D)
        {
            mNodeRRT cNode = (mNodeRRT)TreeNodes[sIndex];
            int c_i = cNode.indexOfFather;
            cNode.update_node_value(goal_pos_3D);
            while (c_i != -1)
            {
                cNode = (mNodeRRT)TreeNodes[c_i];
                cNode.update_node_value(goal_pos_3D);
                c_i = cNode.indexOfFather;
            }
            return;
        }

        public int add_node(Vector3 pos, Vector2 dir, float radius, int indexOfFather)
        {
            mNodeRRT new_node = new mNodeRRT(pos, dir);
            new_node.shapeRadius = radius;
            new_node.indexOfFather = indexOfFather;
            new_node.node_index_inTree = TreeNodes.Count;
            //region
            mRegionManager.addNodeToRegion(new_node);
            //hash
            //            addNodeToHash(new_node, false);

            Vector3 goal_pos_3D = new Vector3(goal_point[0], pos[1], goal_point[1]);
            float dis_toGoal = (pos - goal_pos_3D).magnitude;

            if (indexOfFather >= 0)
            {
                mNodeRRT mParent = (mNodeRRT)TreeNodes[indexOfFather];
                restart_parent_node_values(mParent.node_index_inTree, goal_pos_3D);//for huristic
                mParent.child_nodes_index.Add(TreeNodes.Count);//because count is 1 point further than the indices

                //mParent.cost_to_reach + (new_node.objectPosition - mParent.objectPosition).magnitude;
                new_node.cost_to_reach = new_node.getCostToReach(TreeNodes, numberOfUpdatingRoot + numberOfUpdatingObstacles + numberOfUpdatingStr);//path value
            }

            new_node.node_value = dis_toGoal;//TODO change this value for heuristic
            TreeNodes.Add(new_node);

            if (dis_toGoal < radius && !flag_find_path)
            {
                flag_find_path = true;
                //add the goal state as final node
                mNodeRRT connecting_node_ToGoal = new mNodeRRT(goal_pos_3D, (goal_pos_3D - pos) / ((goal_pos_3D - pos).magnitude + 0.000001f));

                connecting_node_ToGoal.shapeRadius = radius;
                connecting_node_ToGoal.indexOfFather = new_node.node_index_inTree;
                connecting_node_ToGoal.node_index_inTree = TreeNodes.Count;
                connecting_node_ToGoal.node_value = goal_value;

                //new_node.cost_to_reach + (new_node.objectPosition - connecting_node_ToGoal.objectPosition).magnitude;
                connecting_node_ToGoal.cost_to_reach = connecting_node_ToGoal.getCostToReach(TreeNodes, numberOfUpdatingRoot + numberOfUpdatingObstacles + numberOfUpdatingStr);//path value
                //region
                mRegionManager.addNodeToRegion(connecting_node_ToGoal);
                //hash
                //               addNodeToHash(connecting_node_ToGoal, false);

                new_node.child_nodes_index.Add(connecting_node_ToGoal.node_index_inTree);
                TreeNodes.Add(connecting_node_ToGoal);

                goal_index_inTree = connecting_node_ToGoal.node_index_inTree;
                //update the value of the path
                path_from_root = update_parent_values(goal_index_inTree, goal_value);
            }
            return new_node.node_index_inTree;
        }

        public ArrayList update_parent_values(int node_index, float updated_value) // updating value of whole nodes in the path to the root
        {
            mNodeRRT cNode = (mNodeRRT)TreeNodes[node_index];

            float l_path_cost = 0;
            if (updated_value == goal_value)
            {
                cur_cost_path = cNode.getCostToReach(TreeNodes, numberOfUpdatingObstacles + numberOfUpdatingRoot + numberOfUpdatingStr)
                    + (cNode.objectPosition - (new Vector3(goal_point[0], cNode.objectPosition[1], goal_point[1]))).magnitude;
                //if the point we want to reach is non reachable at the moment after each taking sample increase the area
                if (cur_cost_path == not_reaching_ToGoal_value)
                {
                    flag_calculate_path_cost = true;
                }
                else
                {// otherwise do the same as informed RRT*
                    cost_path_inside_notReachArea = 0;
                    flag_calculate_path_cost = false;
                }

                //for simulation part
                if (TreeNodes.Count >= mSimulation_numberOfNodes && cur_cost_path < float.MaxValue) //for the simulation part
                {
                    float new_cost_to_be_added = cost_of_taken_path + cur_cost_path;
                    if (cost_of_found_paths.Count > 0)
                    {
                        float last_added_cost = (float)cost_of_found_paths[cost_of_found_paths.Count - 1];
                        if (Math.Abs(last_added_cost - new_cost_to_be_added) > cRadius / 2)
                        {
                            cost_of_found_paths.Add(new_cost_to_be_added);
                            numberOfChangedInPath++;
                        }
                    }
                    else
                    {
                        cost_of_found_paths.Add(new_cost_to_be_added);
                    }
                }
            }

            mNodeRRT ccNode = cNode;

            int c_i = cNode.indexOfFather;
            cNode.node_value = updated_value;
            ArrayList updated_path = new ArrayList();
            while (c_i != -1)
            {
                cNode = (mNodeRRT)TreeNodes[c_i];

                l_path_cost += (ccNode.objectPosition - cNode.objectPosition).magnitude;

                //we alwayse need to have a path :)
                updated_path.Add(cNode.node_index_inTree);
                cNode.node_value = updated_value;

                ccNode = cNode;
                c_i = cNode.indexOfFather;
            }

            //if the point we want to reach is non reachable at the moment after each taking sample increase the area
            if (flag_calculate_path_cost)
            {
                best_cost_path = l_path_cost + (float)cost_path_inside_notReachArea * (1 / 100f);
            }

            return updated_path;
        }

        //search using heuristic and value of path to find a path to goal on the tree
        public int find_best_path(int standing_index)//standing_index is the index that the ball is standing on that pos at the moment
        {
            if (TreeNodes == null)
                return -1;
            if (TreeNodes.Count < mSimulation_numberOfNodes) //for the simulation part
            {
                return -1;
            }
            if (TreeNodes.Count > 0)
            {
                Vector3 goal_point_3d = new Vector3(goal_point[0], cRadius, goal_point[1]);
                int index_cur_path = current_root;
                mNodeRRT cNode = (mNodeRRT)TreeNodes[index_cur_path];
                bool find_next_step = false;
                int next_step_index = -1;
                bool flag_continue = true;

                if ((cNode.objectPosition - (new Vector3(goal_point[0], cNode.objectPosition[1], goal_point[1]))).magnitude < cRadius / 2)
                {
                    flag_continue = false;
                    goal_index_inTree = index_cur_path;
                    flag_find_path = true;
                }
                int counter_k_step_forward = 0;
                ArrayList created_path_soFar = new ArrayList();
                while (cNode.child_nodes_index.Count > 0 && flag_continue)
                {
                    if (flag_find_path)
                    {
                        if ((cNode.objectPosition - (new Vector3(goal_point[0], cNode.objectPosition[1], goal_point[1]))).magnitude < cRadius / 2)//if accuracy is not less than radius then it is acceptable
                            flag_continue = false;
                    }
                    if (flag_continue)
                    {
                        int index_best_child = -1;
                        float best_value = float.MaxValue;
                        for (int i = 0; i < cNode.child_nodes_index.Count; i++)
                        {
                            mNodeRRT sNode = (mNodeRRT)TreeNodes[(int)cNode.child_nodes_index[i]];
                            if (sNode.getNodeWorth(TreeNodes, numberOfUpdatingRoot + numberOfUpdatingObstacles + numberOfUpdatingStr, numberOfUpdatingGoals, goal_point_3d) < best_value)
                            {
                                best_value = sNode.getNodeWorth(TreeNodes, numberOfUpdatingRoot + numberOfUpdatingObstacles + numberOfUpdatingStr, numberOfUpdatingGoals, goal_point_3d);
                                index_best_child = (int)cNode.child_nodes_index[i];
                            }
                        }
                        if (index_best_child != -1)
                        {
                            mNodeRRT bNode = (mNodeRRT)TreeNodes[index_best_child];
                            Debug.DrawLine(cNode.objectPosition, bNode.objectPosition, Color.red);
                            cNode = bNode;
                            created_path_soFar.Add(cNode);

                            if (index_cur_path == standing_index)
                            {
                                if (!find_next_step)
                                {
                                    find_next_step = true;
                                    next_step_index = index_best_child;
                                }
                            }
                            index_cur_path = index_best_child;
                        }
                        else
                        {
                            cNode.node_value = not_reaching_ToGoal_value;
                            flag_continue = false;// there is no child to check
                        }
                    }
                    counter_k_step_forward++;
                    if (counter_k_step_forward > max_k_step_forward)
                    {
                        flag_continue = false;
                    }
                }//while
                //if (!((cNode.objectPosition - goal_point_3d).magnitude < cNode.shapeRadius))//check if we reach goal
                //    if (cNode.child_nodes_index.Count == 0)//if current node is not goal and does not have any children so it is not our desired path
                //        cNode.node_value = not_reaching_ToGoal_value;//cNode.node_value;

                if (find_next_step && !flag_find_path)
                {
                    mNodeRRT rNode = (mNodeRRT)TreeNodes[current_root];//current pos
                    mNodeRRT endup_bNodeOffer = null;
                    if (best_path_soFar.Count > 0)
                        endup_bNodeOffer = (mNodeRRT)best_path_soFar[best_path_soFar.Count - 1];//best offer so far
                    mNodeRRT endup_nNodeOffer = (mNodeRRT)created_path_soFar[created_path_soFar.Count - 1];
                    mNodeRRT endup_check_with_root = endup_nNodeOffer;
                    if (endup_bNodeOffer == null)
                    {
                        best_path_soFar = created_path_soFar;
                    }
                    else
                    {
                        if ((endup_nNodeOffer.objectPosition - goal_point_3d).magnitude < (endup_bNodeOffer.objectPosition - goal_point_3d).magnitude)
                        {
                            best_path_soFar = created_path_soFar;
                        }
                        else
                        {
                            endup_check_with_root = endup_bNodeOffer;
                        }
                    }
                    if ((endup_check_with_root.objectPosition - goal_point_3d).magnitude > (rNode.objectPosition - goal_point_3d).magnitude)
                    {
                        next_step_index = rNode.node_index_inTree;
                    }
                    else
                    {
                        mNodeRRT nextNode = (mNodeRRT)best_path_soFar[0];
                        if (nextNode.node_index_inTree == current_root)
                        {
                            best_path_soFar.RemoveAt(0);
                            if (best_path_soFar.Count > 0)
                                nextNode = (mNodeRRT)best_path_soFar[0];
                            else
                                nextNode = rNode;
                        }
                        next_step_index = nextNode.node_index_inTree;
                    }
                }
                cNode = (mNodeRRT)TreeNodes[standing_index];
                if (!find_next_step)// either the ball is standing at a node without any children or the new best path is not passing from this node
                {
                    if ((cNode.objectPosition - (new Vector3(goal_point[0], cNode.objectPosition[1], goal_point[1]))).magnitude < cNode.shapeRadius)//check if we reach goal
                        find_next_step = true;
                    else
                    {
                        next_step_index = cNode.indexOfFather;
                        //TODO: if the new path is not passing current node do something other than setting next node to father
                    }
                }
                return next_step_index;
            }
            return -1;
        }

        public void change_goal_point(Vector2 new_goal)
        {
            if ((goal_point - new_goal).magnitude > cRadius / 2)//accuracy
            {
                numberOfUpdatingGoals++;
                flag_find_path = false;
                goal_index_inTree = -1;

                set_goal_point(new_goal[0], new_goal[1]);
                cur_cost_path = float.MaxValue;
                best_cost_path = float.MaxValue;
                cost_path_inside_notReachArea = 0;
                flag_calculate_path_cost = false;

                //For simulation
                cost_of_found_paths = new ArrayList();
                numberOfChangedInPath = 0;//during the movement the path is going to change a couple of times, this will tell how many times it changed
                iteration_needed_for_firstPath = 0;//iteration needed for returning the first path
                cost_of_taken_path = 0;//iteration needed for returning the first path
            }
            return;
        }

        public void set_goal_point(float x, float z)//return best path from last tree & use the same nodes in the tree to build the new one
        {
            Vector3 last_goal = new Vector3(goal_point[0], cRadius, goal_point[1]);
            Vector3 goal_3D_pos = new Vector3(x, cRadius, z);

            bool isGoalChanged = false;

            RaycastHit hit;
            //Camera.main.ScreenPointToRay(current_goal);
            Vector3 p1 = goal_3D_pos;
            Vector3 p2 = new Vector3(goal_3D_pos[0], -goal_3D_pos[1], goal_3D_pos[2]);
            if (Physics.Raycast(p1, (p2 - p1) / (p2 - p1).magnitude, out hit, (p2 - p1).magnitude))
            {
                Transform objectHit = hit.transform;
                if (objectHit.name == "Plane")
                {
                    goal_point[0] = x;
                    goal_point[1] = z;
                    isGoalChanged = true;
                }
            }
            //goal_point[0] = x;
            //goal_point[1] = z;
            if (TreeNodes == null)
            {
                return;
            }
            if (TreeNodes.Count > 0 && isGoalChanged)
            {
                ArrayList check_goal_points = new ArrayList();
                check_goal_points.Add(last_goal);
                check_goal_points.Add(goal_3D_pos);
                ArrayList arr_closest_index = new ArrayList();
                ArrayList arr_min_dis = new ArrayList();
                mRegionManager.find_closest_nodes_ToGoals(check_goal_points, arr_closest_index, arr_min_dis);

                //                float min_dis_last_goal = (float)arr_min_dis[0];
                //                int index_min_dis_lGoal = (int)arr_closest_index[0];

                float min_dis = (float)arr_min_dis[1];
                int index_min_dis = (int)arr_closest_index[1];
                if (index_min_dis != -1)
                {
                    mNodeRRT closest_node = (mNodeRRT)TreeNodes[index_min_dis];
                    if (!collisionDetection2(closest_node.objectPosition, goal_3D_pos) && closest_node.reachable_node)
                    //getCostToReach(TreeNodes, numberOfUpdatingStr+numberOfUpdatingRoot+numberOfUpdatingObstacles) < not_reaching_ToGoal_value)
                    {
                        int update_node_index = index_min_dis;
                        if (min_dis > cRadius / 2)
                        {
                            Vector3 dir = (goal_3D_pos - closest_node.objectPosition) / ((goal_3D_pos - closest_node.objectPosition).magnitude + 0.000001f);
                            add_node(goal_3D_pos, new Vector2(dir[0], dir[2]), cRadius, index_min_dis);
                        }
                        else
                        {
                            path_from_root = update_parent_values(update_node_index, goal_value);
                            flag_find_path = true;
                            goal_index_inTree = update_node_index;
                        }
                    }
                }
            }
            return;
        }
        public bool collisionDetection2(Vector3 p1, Vector3 p2)
        {
            RaycastHit collisionInfo;

            Vector3 dir = p2 - p1;
            //bool result = !Physics.Raycast(p1, dir, out collisionInfo, dir.magnitude, layer);
            //bool result = !Physics.Raycast(p1, dir, out collisionInfo, dir.magnitude, layer);
            bool result = Physics.SphereCast(p1, ballRadius, dir, out collisionInfo, dir.magnitude, fixedObstaclesLayer);

#if _DEBUG_1_
		Debug.DrawRay(this.q, dir, Color.red);
#endif
            return result;
        }
        public bool collisionDetection(Vector3 p1, Vector3 p2)//p1 and p2 represent the center of mass of the agent
        {
            RaycastHit mhitinfo;
            bool isHit = Physics.Raycast(p1, (p2 - p1) / (p2 - p1).magnitude, out mhitinfo, (p2 - p1).magnitude);
            Vector3 XPlus = new Vector3(ballRadius, 0f, 0f);
            Vector3 ZPlus = new Vector3(0f, 0f, ballRadius);
            float dis_val = (p2 - p1).magnitude + 0.00001f;
            Vector3 dir_p1top2 = (p2 - p1) / dis_val;
            if (!isHit)
            {
                isHit = Physics.Raycast(p1 + dir_p1top2 * cRadius, dir_p1top2, out mhitinfo, dis_val);
            }
            if (!isHit)
            {
                isHit = Physics.Raycast(p1 + XPlus, dir_p1top2, out mhitinfo, dis_val);
            }
            if (!isHit)
            {
                isHit = Physics.Raycast(p1 - XPlus, dir_p1top2, out mhitinfo, dis_val);
            }
            if (!isHit)
            {
                isHit = Physics.Raycast(p1 + ZPlus, dir_p1top2, out mhitinfo, dis_val);
            }
            if (!isHit)
            {
                isHit = Physics.Raycast(p1 - ZPlus, dir_p1top2, out mhitinfo, dis_val);
            }
            // TODO: add *sample line* to enable the agent pass objects with suitable property
            //       mhitinfo.collider.transform.position.x;
            return isHit;
        }

        public Vector2 Uniform_Sampling_stateSpace(Vector2 regionX, Vector2 regionZ)
        {
            float r_x = UnityEngine.Random.Range(regionX[0], regionX[1]);
            float r_z = UnityEngine.Random.Range(regionZ[0], regionZ[1]);
            return new Vector2(r_x, r_z);
        }

        public Vector2 Uniform_Sampling_ellipse(Vector2 S_Point, Vector2 E_Point, float c_best)//implemantation of informed RRT*
        {
            float distance_SToE = (S_Point - E_Point).magnitude;
            float c_min = distance_SToE;

            float r_inside_ball = UnityEngine.Random.Range(0f, 1f);
            float theta_inside_ball = (float)(2 * Math.PI) * UnityEngine.Random.Range(0f, 1f);
            float r2 = (c_best * c_best) - (c_min * c_min);
            if (r2 < 0)
                r2 = 0;
            Vector2 sample_in_ellipse_coordinate = new Vector2((float)(r_inside_ball * Math.Cos(theta_inside_ball) * (c_best / 2f))
                , (float)(r_inside_ball * Math.Sin(theta_inside_ball) * ((Math.Sqrt(r2)) / 2)));//(float)Math.Sqrt((c_best*c_best)-(c_min*c_min))/2f);

            Vector2 dir_StoE = (E_Point - S_Point) / (distance_SToE + 0.00001f);
            double theta_rotation = Math.Atan2(dir_StoE[1], dir_StoE[0]);
            double rotated_x = Math.Cos(theta_rotation) * sample_in_ellipse_coordinate[0] - Math.Sin(theta_rotation) * sample_in_ellipse_coordinate[1];
            double rotated_y = Math.Sin(theta_rotation) * sample_in_ellipse_coordinate[0] + Math.Cos(theta_rotation) * sample_in_ellipse_coordinate[1];

            Vector2 center_SandE = (S_Point + E_Point) / 2f;

            Vector2 n_sample = new Vector2((float)rotated_x, (float)rotated_y) + center_SandE;//new Vector2((float)rotated_x, (float)rotated_y);
            if (FlagUseColor)
                Debug.DrawLine((new Vector3(center_SandE[0], 0.5f, center_SandE[1])), (new Vector3(n_sample[0], 0.5f, n_sample[1])), Color.blue);

            return n_sample;
        }
        public bool iterationsExceeded()
        {
            return iterations >= maxIterations;
        }
        public bool finishedGrowingTree()
        {
            return finishedGrowingTree_;
        }
        public int getIterations()
        {
            return iterations;
        }
        public void sampleSpace(bool check_nodes_OnTree, bool flag_continue_time) //sample in 2D environment
        {

            bool flag_informed_RRTStar = FlagInformedSampling;
            float ratio_of_informed_sampling = 0.5f;
            // if it is not sampling based on RT-RRT*, and only based on informed RRT*
            if (FlagInformedSampling && !FlagRewiringCircle)
            {
                ratio_of_informed_sampling = 1.0f;
            }
            float r_n = UnityEngine.Random.Range(0f, 1f);
            Vector2 sample_point = new Vector2();
            bool flag_set_sample_point = false;
            if (r_n < rateTowardGoal)
            {
                //generate a greedy sample toward the goal
                sample_point = new Vector2(goal_point[0], goal_point[1]);
                flag_set_sample_point = true;
            }
            else
            {
                if (Math.Abs(cur_cost_path - best_cost_path) > 0 && cur_cost_path < float.MaxValue)//if path is changed
                    best_cost_path = cur_cost_path;
                //generate a sample in the space (which is 2D space)
                Vector2 m_regionX = new Vector2(min_range_x, max_range_x);
                Vector2 m_regionZ = new Vector2(min_range_z, max_range_z);
                bool flag_uniform_sampling_stateSpace = false;
                // perform informed sampling inside an elips
                if (best_cost_path < float.MaxValue && flag_informed_RRTStar && best_cost_path > cRadius && r_n < ratio_of_informed_sampling)
                {
                    mNodeRRT root_node = (mNodeRRT)TreeNodes[current_root];
                    sample_point = Uniform_Sampling_ellipse(new Vector2(root_node.objectPosition[0], root_node.objectPosition[2]), goal_point, best_cost_path);
                    flag_set_sample_point = true;
                    if (sample_point[0] < m_regionX[0] || sample_point[0] > m_regionX[1] || sample_point[1] < m_regionZ[0] || sample_point[1] > m_regionZ[1])
                    {
                        flag_uniform_sampling_stateSpace = true;
                        flag_set_sample_point = false;
                    }
                    //if the point we want to reach is non reachable at the moment after each taking sample increase the area
                    if (flag_calculate_path_cost && flag_set_sample_point)
                        cost_path_inside_notReachArea = best_cost_path;
                }
                else
                    flag_uniform_sampling_stateSpace = true;

                // an unform sample inside the region
                if (flag_uniform_sampling_stateSpace)
                {
                    sample_point = Uniform_Sampling_stateSpace(m_regionX, m_regionZ);
                    flag_set_sample_point = true;
                }
            }
            if (!flag_set_sample_point) return;
            //find the closest node in tree to the sample point and move toward it (just one step)
            int index_closest_node = mRegionManager.find_closest_node_inTree(sample_point);
            if (index_closest_node == -1) return;

            //generate a random number
            float r_s = UnityEngine.Random.Range(0f, 1f);
            //get a sample in the line between the closest node in tree and sample_point
            mNodeRRT closestNode = (mNodeRRT)TreeNodes[index_closest_node];
            Vector2 subtract_S_C = (sample_point - new Vector2(closestNode.objectPosition[0], closestNode.objectPosition[2]));
            Vector3 new_sample_pos = new Vector3(closestNode.objectPosition[0] + r_s * (subtract_S_C[0]),
                closestNode.objectPosition[1]/*same height as closes sample TODO: change it with sampleLine*/,
                closestNode.objectPosition[2] + r_s * (subtract_S_C[1]));
            if ((sample_point - goal_point).magnitude > 0)
            {
                new_sample_pos = new Vector3(sample_point[0],
                closestNode.objectPosition[1]/*same height as closes sample TODO: change it with sampleLine*/,
                sample_point[1]);
            }
            iterations++;
            if (iterationsExceeded())
            {
                if (!finishedGrowingTree_)
                    iterationsToFinish = iterations;
                finishedGrowingTree_ = true;

            }
            if (!collisionDetection2(closestNode.objectPosition, new_sample_pos))
            {
                if (finishedGrowingTree())
                {

                    update_tree_fathers(new_sample_pos, 5, index_closest_node, false, false, flag_continue_time);

                }
                else
                {

                    update_tree_fathers(new_sample_pos, 5, index_closest_node, true, false, flag_continue_time);
                }
            }

            //find node inside tree that particle filter suggests
            if (check_nodes_OnTree)
            {
                mNodeRRT onTreeNode = (mNodeRRT)TreeNodes[current_root];
                update_tree_fathers(onTreeNode.objectPosition, 5, current_root, false, true, flag_continue_time);
            }
            return;
        }

        public void update_cost_to_reach(int standing_node)//increase the number of needing updates for costs and change the structure of tree
        {
            //difference between this variable and numberOfupdateCost in each node tells us that we need another update for cost or not
            numberOfUpdatingRoot++;//this variable is increased every time that root of the tree has changed

            Vector3 goal_pos_3D = new Vector3(goal_point[0], cRadius, goal_point[1]);

            mNodeRRT sNode = (mNodeRRT)TreeNodes[standing_node];
            sNode.cost_to_reach = 0;
            sNode.numberOfUpdates_cost = numberOfUpdatingRoot + numberOfUpdatingObstacles + numberOfUpdatingStr;//because I update this part when I change the cost_to_reach value
            //queue_onTree_nodes.restart_queue(sNode);//when we should update cost of nodes so we should change structure from begining
            queue_onTree_nodes.insertNodeFirst(sNode);//when we should update cost of nodes so we should change structure from begining

            //make standing node root of the tree
            current_root = sNode.node_index_inTree;
            mNodeRRT cNode = sNode;
            if (cNode.indexOfFather != -1)
            {
                mNodeRRT fNode = (mNodeRRT)TreeNodes[cNode.indexOfFather];
                int current_index_fatherOfFather = -3;//this initialization is not important
                do
                {
                    fNode.child_nodes_index.Remove(cNode.node_index_inTree);
                    cNode.child_nodes_index.Add(fNode.node_index_inTree);

                    current_index_fatherOfFather = fNode.indexOfFather;
                    fNode.indexOfFather = cNode.node_index_inTree;

                    if (current_index_fatherOfFather != -1)
                    {
                        cNode = fNode;
                        fNode = (mNodeRRT)TreeNodes[current_index_fatherOfFather];
                    }
                    if (fNode.getNode_Value(goal_pos_3D, numberOfUpdatingGoals) < 0)//restart goal value for father nodes
                        fNode.update_node_value(goal_pos_3D);
                }
                while (current_index_fatherOfFather != -1);
            }
            sNode.indexOfFather = -1;
        }

        public void update_costs_basedOn_movingObstacles(ArrayList obstacles_pos)/////////////// TODO: check not only nodes but path between nodes as well
        {
            //            float raduis_check_obstacle_region = 5;
            if (cur_pos_obstacles == null)
            {
                cur_pos_obstacles = new ArrayList();
            }
            if (cur_pos_obstacles.Count < obstacles_pos.Count)
            {
                int diff_num_obs = obstacles_pos.Count - cur_pos_obstacles.Count;
                for (int i = 0; i < diff_num_obs; i++)
                {
                    cur_pos_obstacles.Add(new Vector3(not_reaching_ToGoal_value, not_reaching_ToGoal_value, not_reaching_ToGoal_value));
                }
            }

            ArrayList pass_CPos_obstacle_inside_check_region = new ArrayList();
            ArrayList pass_LPos_obstacle_inside_check_region = new ArrayList();
            ArrayList index_changed_curPos_obstacles = new ArrayList();
            for (int i = 0; i < obstacles_pos.Count; i++)
            {
                Vector3 c_obstacle_pos_i = (Vector3)obstacles_pos[i];
                Vector3 l_obstacle_pos_i = (Vector3)cur_pos_obstacles[i];
                if ((c_obstacle_pos_i - cur_pos_agent).magnitude < radius_check_obstacle_region + 2 * cRadius)
                {
                    pass_CPos_obstacle_inside_check_region.Add(c_obstacle_pos_i);
                    pass_LPos_obstacle_inside_check_region.Add(l_obstacle_pos_i);
                    index_changed_curPos_obstacles.Add(i);
                }
            }
            float m_collide_region = radius_contact_region;
            bool update_path_toGoal = false;
            //            float i_accuracy = cRadius / 4f;//////////////////////////////////////////////////////////////////////important
            //numberOfUpdatingObstacles++ if necessary
            //            int l_update_obstacles = numberOfUpdatingObstacles;

            float blocked_node_cost_dis = float.MaxValue;

            numberOfUpdatingObstacles = mRegionManager.update_cost_to_reach_for_dynamicObstacles(TreeNodes, pass_CPos_obstacle_inside_check_region, pass_LPos_obstacle_inside_check_region
                , numberOfUpdatingObstacles, current_root, m_collide_region, ref update_path_toGoal, goal_index_inTree
                , numberOfUpdatingObstacles + numberOfUpdatingRoot + numberOfUpdatingStr, index_changed_curPos_obstacles, ref blocked_node_cost_dis);

            //restart traversing on the tree whenever a dynamic obstacle moves
            if (blocked_node_cost_dis < c_distance_ring)
            {
                mNodeRRT c_root_node = (mNodeRRT)TreeNodes[current_root];
                //queue_onTree_nodes.restart_queue(c_root_node);////////////////TODO: restart Q
                queue_onTree_nodes.insertNodeFirst(c_root_node);
                c_distance_ring = 0;
                numberOfUpdatingStr++;
            }

            //update current pos of every obstacle
            for (int i = 0; i < obstacles_pos.Count; i++)
            {
                cur_pos_obstacles[i] = obstacles_pos[i];
            }

            if (update_path_toGoal)
            {
                restart_value_goal_path();
                path_from_root = update_parent_values(goal_index_inTree, goal_value);
            }
            return;
        }

        private void delete_child_inTrees(int fatherIndex, int childIndex)
        {//just delete index of that child in the parent. the node is still out there and can be connected to another node in the tree
            if (fatherIndex >= 0)
                ((mNodeRRT)TreeNodes[fatherIndex]).child_nodes_index.Remove(childIndex);
            return;
        }

        private ArrayList fathersOfNode(int index_node, int max_counter)// returns max_counter fathers of index_node in the tree
        {
            int counter = 0;
            mNodeRRT iNode = (mNodeRRT)TreeNodes[index_node];
            ArrayList fNodes = new ArrayList();
            bool flag_continue = true;
            mNodeRRT cNode = iNode;
            while (flag_continue)
            {
                if (cNode.indexOfFather == -1)
                    flag_continue = false;
                if (flag_continue)
                {
                    mNodeRRT nNode = (mNodeRRT)TreeNodes[cNode.indexOfFather];
                    if (counter < max_counter)
                    {
                        fNodes.Add(cNode.indexOfFather);
                        cNode = nNode;
                    }
                    else
                    {
                        flag_continue = false;
                    }
                    counter++;
                }
            }
            return fNodes;
        }

        private ArrayList fathersOfNode(int index_node, float l_search_ball)// returns fathers of index_node which are inside search ball in the tree
        {
            mNodeRRT iNode = (mNodeRRT)TreeNodes[index_node];
            ArrayList fNodes = new ArrayList();
            bool flag_continue = true;
            mNodeRRT cNode = iNode;
            while (flag_continue)
            {
                if (cNode.indexOfFather == -1)
                    flag_continue = false;
                if (flag_continue)
                {
                    mNodeRRT nNode = (mNodeRRT)TreeNodes[cNode.indexOfFather];
                    float c_dis = (nNode.objectPosition - iNode.objectPosition).magnitude;
                    if (c_dis < l_search_ball)
                    {
                        fNodes.Add(cNode.indexOfFather);
                        cNode = nNode;
                    }
                    else
                    {
                        flag_continue = false;
                    }
                }
            }
            return fNodes;
        }

        private bool CheckLoop_fathersOfNode(int index_node, float l_search_ball)// returns fathers of index_node which are inside search ball in the tree
        {
            mNodeRRT iNode = (mNodeRRT)TreeNodes[index_node];
            ArrayList fNodes = new ArrayList();
            bool flag_continue = true;
            mNodeRRT cNode = iNode;
            bool flag_loop = false;
            while (flag_continue)
            {
                if (cNode.indexOfFather == -1)
                    flag_continue = false;
                if (flag_continue)
                {
                    mNodeRRT nNode = (mNodeRRT)TreeNodes[cNode.indexOfFather];
                    float c_dis = (nNode.objectPosition - iNode.objectPosition).magnitude;
                    if (c_dis < l_search_ball)
                    {
                        if (!fNodes.Contains(cNode.indexOfFather))
                        {
                            fNodes.Add(cNode.indexOfFather);
                            cNode = nNode;
                        }
                        else
                        {
                            flag_continue = false;
                            flag_loop = true;
                        }
                    }
                    else
                    {
                        flag_continue = false;
                    }
                }
            }
            return flag_loop;
        }

        public void restart_value_goal_path()
        {
            Vector3 goal_pos_3D = new Vector3(goal_point[0], cRadius, goal_point[1]);
            for (int i = 0; i < path_from_root.Count; i++)
            {
                mNodeRRT mNode = (mNodeRRT)TreeNodes[(int)path_from_root[i]];
                if (mNode.getNode_Value(goal_pos_3D, numberOfUpdatingGoals) < 0)//restart goal path
                    mNode.update_node_value(goal_pos_3D);
            }
            return;
        }

        //update structure of the tree to find the least cost to reach to every other nodes
        public void update_tree_fathers(Vector3 Znew, int k_near, int index_nearest_node, bool insert_node, bool choose_from_tree, bool flag_continue_time)
        {
            int numberOfcheckingNodes = k_near;
            float A_total = (max_range_x - min_range_x) * (max_range_z - min_range_z);
            int n_total = TreeNodes.Count;
            float search_ball = (float)Math.Sqrt(((float)numberOfcheckingNodes * A_total) / (Math.PI * (float)n_total));// this is same as radius of agent or ball
            if (search_ball < cRadius)
                search_ball = cRadius;

            if (k_near < 1)//we should find at least one near node
                k_near = 1;

            restart_value_goal_path();

            int k_near_counter = 0;
            ArrayList min_index_neighbor = new ArrayList();
            int min_index = -1;
            if (!choose_from_tree)
            {
                min_index = mRegionManager.find_Neighbors_inSearchBall_inTree(Znew, search_ball, min_index_neighbor, ref k_near_counter);
                if (min_index == -1)
                    return;

                ///////////////////////////////this part controls the density of the tree/////////////////////////////////////
                /*
                 if (iterations < maxIterations && ( k_near_counter < k_near || (((mNodeRRT)TreeNodes[min_index]).objectPosition - Znew).magnitude > cRadius / 2))
                     insert_node = true;
                 else
                     insert_node = false;
                */
                //////////////////////////end of this part controls the density of the tree///////////////////////////////////
            }
            else
            {
                min_index = index_nearest_node;
            }
            //connect new node to the tree or just select the closet one
            int nParentNode_index = -1;
            if (insert_node)
            {
                bool use_closest_node = true;
                if (min_index_neighbor.Count > 0 && FlagRewireTree)//choose a father for znew and change father of other index 1 to k_near if it is necessary
                {
                    //choose father for zNew
                    float min_value_node = float.MaxValue;
                    int index_min_value_father_zNew = -1;
                    for (int k = 0; k < min_index_neighbor.Count; k++)
                    {
                        mNodeRRT kNode = (mNodeRRT)TreeNodes[(int)min_index_neighbor[k]];

                        //change the value of cost_to_reach based on the current structure before using kNode.cost_to_reach
                        float zNew_value = kNode.getCostToReach(TreeNodes, numberOfUpdatingRoot + numberOfUpdatingObstacles + numberOfUpdatingStr) + (Znew - kNode.objectPosition).magnitude;
                        bool f_collision = collisionDetection2(kNode.objectPosition, Znew);
                        if (zNew_value < min_value_node && !f_collision)
                        {
                            min_value_node = zNew_value;
                            index_min_value_father_zNew = (int)min_index_neighbor[k];
                        }
                    }
                    if (index_min_value_father_zNew != -1)
                    {
                        mNodeRRT fNode = (mNodeRRT)TreeNodes[index_min_value_father_zNew];
                        Vector3 dir_to_Znew = (Znew - fNode.objectPosition) / ((Znew - fNode.objectPosition).magnitude + 0.0000001f);
                        nParentNode_index = add_node(Znew, new Vector2(dir_to_Znew[0], dir_to_Znew[2]), fNode.shapeRadius, fNode.node_index_inTree);
                        use_closest_node = false;
                    }
                }
                if (use_closest_node)
                {
                    mNodeRRT closestNode = (mNodeRRT)TreeNodes[min_index];
                    Vector3 dir_to_Znew = (Znew - closestNode.objectPosition) / ((Znew - closestNode.objectPosition).magnitude + 0.0000001f);
                    nParentNode_index = add_node(Znew, new Vector2(dir_to_Znew[0], dir_to_Znew[2]), closestNode.shapeRadius, closestNode.node_index_inTree);//TODO: change direction, radius
                }
                mNodeRRT m_new_node = (mNodeRRT)TreeNodes[nParentNode_index];
                m_new_node.nodes_inside_search_ball = min_index_neighbor;
                for (int k = 0; k < min_index_neighbor.Count; k++)
                {
                    mNodeRRT kNode = (mNodeRRT)TreeNodes[(int)min_index_neighbor[k]];
                    kNode.nodes_inside_search_ball.Add(m_new_node.node_index_inTree);
                }
            }
            else
            {
                nParentNode_index = min_index;
            }
            //connect other nodes around the new node such that the new cost must be lower than old cost
            mNodeRRT nParentNode = (mNodeRRT)TreeNodes[nParentNode_index];
            //            ArrayList fathersOfnewNode = new ArrayList();

            Vector3 goal_pos_3D = new Vector3(goal_point[0], Znew[1], goal_point[1]);

            bool flag_continue = true;

            if (!choose_from_tree)
            {
                queue_random_samples.insertNodeFirst(nParentNode);
                if (queue_random_samples.getSize() > TreeNodes.Count)
                    queue_random_samples.restart_queue(nParentNode);
            }
            else
            {
                if (queue_onTree_nodes.check_empty())
                {
                    queue_onTree_nodes.insertNodeFirst(nParentNode);
                    numberOfUpdatingStr++;
                }
                if (queue_onTree_nodes.getSize() > TreeNodes.Count)
                    queue_onTree_nodes.restart_queue(nParentNode);
            }

            ArrayList min_index_Znew = min_index_neighbor;

            int c_t = DateTime.Now.Millisecond;
            int d_t = 0;

            if (!FlagRewireTree)
            {
                flag_continue = false;
                queue_onTree_nodes.restart_queue(nParentNode);
                queue_random_samples.restart_queue(nParentNode);
            }

            if (!FlagRewiringCircle)
            {
                queue_onTree_nodes.restart_queue(nParentNode);
                if (choose_from_tree)
                {
                    flag_continue = false;
                }
            }
            while (flag_continue)// update other nodes as well
            {
                //                bool check_fathers_more_searchBall = false;
                if (!choose_from_tree)
                {
                    nParentNode = queue_random_samples.getFirstNode();
                    if (nParentNode == null) { return; }
                    if (nParentNode.node_index_inTree == min_index && !insert_node)
                    {
                        min_index_neighbor = min_index_Znew;
                        for (int m_a = 0; m_a < nParentNode.nodes_inside_search_ball.Count; m_a++)
                        {
                            if (!min_index_neighbor.Contains((int)nParentNode.nodes_inside_search_ball[m_a]))
                            {
                                min_index_neighbor.Add((int)nParentNode.nodes_inside_search_ball[m_a]);
                            }
                        }
                        //                        check_fathers_more_searchBall = true;
                    }
                    else
                    {
                        min_index_neighbor = nParentNode.nodes_inside_search_ball;
                    }
                }
                else
                {
                    nParentNode = queue_onTree_nodes.getFirstNode();
                    if (nParentNode == null) { return; }
                    min_index_neighbor = nParentNode.nodes_inside_search_ball;
                    if (min_index_neighbor.Count == 0)
                        min_index_neighbor = nParentNode.child_nodes_index;
                }

                for (int k = 0; k < min_index_neighbor.Count; k++)
                {
                    int index_in_kNear_list = (int)min_index_neighbor[k];
                    mNodeRRT cNode = (mNodeRRT)TreeNodes[index_in_kNear_list];

                    if ((cNode.objectPosition - nParentNode.objectPosition).magnitude < search_ball)
                    {
                        mRegionManager.updateReachableNode(cNode, cur_pos_agent, radius_check_obstacle_region, radius_contact_region, cur_pos_obstacles);
                        //change the value of cost_to_reach based on the current structure before using cNode.cost_to_reach
                        float curCost = cNode.getCostToReach(TreeNodes, numberOfUpdatingRoot + numberOfUpdatingObstacles + numberOfUpdatingStr);//cost_to_reach;
                        //change the value of cost_to_reach based on the current structure before using nParentNode.cost_to_reach
                        float newCost = nParentNode.getCostToReach(TreeNodes, numberOfUpdatingRoot + numberOfUpdatingObstacles + numberOfUpdatingStr);
                        if (newCost != float.MaxValue)
                            newCost = newCost + (cNode.objectPosition - nParentNode.objectPosition).magnitude;
                        bool f_collision = collisionDetection2(cNode.objectPosition, nParentNode.objectPosition);

                        //if (!cNode.reachable_node && !f_collision)
                        //{
                        //    int notifyme = 1;
                        //}

                        if (curCost > newCost && !f_collision && cNode.reachable_node)
                        {
                            int Past_father_index = cNode.indexOfFather;
                            delete_child_inTrees(Past_father_index, cNode.node_index_inTree);

                            if (!nParentNode.child_nodes_index.Contains(cNode.node_index_inTree))
                                nParentNode.child_nodes_index.Add(cNode.node_index_inTree);

                            cNode.indexOfFather = nParentNode.node_index_inTree;
                            mNodeRRT fNode = (mNodeRRT)TreeNodes[nParentNode.node_index_inTree];
                            if (FlagUseColor)
                                Debug.DrawLine(cNode.objectPosition + (new Vector3(0f, 0.3f, 0f)), fNode.objectPosition + (new Vector3(0f, 0.3f, 0f)), Color.green);

                            if (CheckLoop_fathersOfNode(cNode.node_index_inTree, 10))//////////////////////////////////////////TODO: check this part!!!!!!!!!!!!!!!!!!!!!!
                            {
                                CheckLoop_fathersOfNode(cNode.node_index_inTree, 10);
                                delete_child_inTrees(nParentNode.node_index_inTree, cNode.node_index_inTree);
                                mNodeRRT Past_parent = (mNodeRRT)TreeNodes[Past_father_index];
                                if (!Past_parent.child_nodes_index.Contains(cNode.node_index_inTree))
                                    Past_parent.child_nodes_index.Add(cNode.node_index_inTree);
                                cNode.indexOfFather = Past_father_index;

                                //float nncurCost = cNode.getCostToReach(TreeNodes, numberOfUpdatingRoot + numberOfUpdatingObstacles + numberOfUpdatingStr);//cost_to_reach;
                                ////change the value of cost_to_reach based on the current structure before using nParentNode.cost_to_reach
                                //float nnnewCost = nParentNode.getCostToReach(TreeNodes, numberOfUpdatingRoot + numberOfUpdatingObstacles + numberOfUpdatingStr);
                                //if (nnnewCost != float.MaxValue)
                                //    nnnewCost = nnnewCost + (cNode.objectPosition - nParentNode.objectPosition).magnitude;
                            }
                            else
                            {
                                //if new child is still worth checking then update fathers value to check the child as well
                                if (cNode.getNode_Value(goal_pos_3D, numberOfUpdatingGoals) != not_reaching_ToGoal_value)
                                {
                                    if (nParentNode.getNode_Value(goal_pos_3D, numberOfUpdatingGoals) == not_reaching_ToGoal_value)// or equals to not_reaching_toGoal_value
                                        nParentNode.update_node_value(goal_pos_3D);
                                }

                                cNode.cost_to_reach = newCost;
                                cNode.numberOfUpdates_cost = numberOfUpdatingObstacles + numberOfUpdatingRoot + numberOfUpdatingStr;

                                if (!choose_from_tree)
                                {
                                    queue_random_samples.addNodeToQueue(cNode);
                                }
                            }
                        }
                        if (choose_from_tree)// traversing the tree
                        {
                            mRegionManager.updateReachableNode(cNode, cur_pos_agent, radius_check_obstacle_region, radius_contact_region, cur_pos_obstacles);
                            if (nParentNode.getCostToReach(TreeNodes, numberOfUpdatingRoot + numberOfUpdatingObstacles + numberOfUpdatingStr) < curCost && nParentNode.reachable_node)
                            {
                                float cost_ring = queue_onTree_nodes.addNodeToQueueForStructure(TreeNodes, cNode, numberOfUpdatingRoot + numberOfUpdatingStr, FlagUseColor);
                                if (cost_ring != -1 && c_distance_ring < cost_ring)
                                {
                                    c_distance_ring = cost_ring;
                                }
                            }
                        }
                    }
                    else
                    {
                        int index_in_search_ball = nParentNode.nodes_inside_search_ball.IndexOf(cNode.node_index_inTree);
                        if (index_in_search_ball >= 0)
                            nParentNode.nodes_inside_search_ball.Remove(index_in_search_ball);
                    }
                }

                d_t += get_delta_miliSec(c_t);
                c_t += get_delta_miliSec(c_t);
                if (d_t > 0 && flag_continue_time)
                {
                    flag_continue = false;
                }
                else
                {
                    flag_continue = flag_continue_time;
                }

                if (!choose_from_tree)
                {
                    if (flag_continue)
                        flag_continue = !queue_random_samples.check_empty();
                }
                else
                {
                    if (flag_continue)
                        flag_continue = !queue_onTree_nodes.check_empty();
                }
            }

            if (flag_find_path)
                path_from_root = update_parent_values(goal_index_inTree, goal_value);
            return;
        }

        int get_delta_miliSec(int cur_mili_sec)
        {
            int n_milisec = DateTime.Now.Millisecond;
            int delta_sec = n_milisec - cur_mili_sec;
            if (delta_sec < 0)
                delta_sec = 1000 - cur_mili_sec + n_milisec;
            return delta_sec;
        }

        public void drawTree()
        {
            if (!flag_debug_on)
                return;
            for (int i = 0; i < TreeNodes.Count; i++)
            {
                mNodeRRT mN = (mNodeRRT)TreeNodes[i];
                if (mN.cost_to_reach != not_reaching_ToGoal_value && mN.reachable_node)
                {
                    for (int j = 0; j < mN.child_nodes_index.Count; j++)
                    {
                        mNodeRRT mN_child = (mNodeRRT)TreeNodes[(int)mN.child_nodes_index[j]];
                        if (mN_child.cost_to_reach != not_reaching_ToGoal_value && mN_child.reachable_node)
                            Debug.DrawLine(mN.objectPosition - (new Vector3(0f, 0.3f, 0f)), mN_child.objectPosition - (new Vector3(0f, 0.3f, 0f)), Color.black);
                    }
                }
                else if (mN.reachable_node)
                {
                    for (int j = 0; j < mN.child_nodes_index.Count; j++)
                    {
                        mNodeRRT mN_child = (mNodeRRT)TreeNodes[(int)mN.child_nodes_index[j]];
                        if (mN_child.reachable_node && FlagUseColor)
                            Debug.DrawLine(mN.objectPosition - (new Vector3(0f, 0.3f, 0f)), mN_child.objectPosition - (new Vector3(0f, 0.3f, 0f)), Color.yellow);
                    }
                }
            }
            return;
        }

        private void sampleLine() { /*TODO*/}//sample in 1D environment which adds some nodes to the

        public Vector3 setNextTargetPos(Vector3 curPos, ref Vector3 targetPos, ref int cur_index_on_RRT, int next_index_onRRT)
        {
            cur_pos_agent = curPos;
            mNodeRRT mCurNode = (mNodeRRT)TreeNodes[cur_index_on_RRT];
            if (next_index_onRRT != -1)
            {
                mNodeRRT mNextNode = (mNodeRRT)TreeNodes[next_index_onRRT];
                if ((curPos - targetPos).magnitude < mCurNode.shapeRadius)
                {
                    //simulation part
                    cost_of_taken_path += (mCurNode.objectPosition - mNextNode.objectPosition).magnitude;
                    // algorithm part
                    targetPos = mNextNode.objectPosition;
                    cur_index_on_RRT = next_index_onRRT;
                    update_cost_to_reach(cur_index_on_RRT);
                    path_length_onTree = 0;
                }
                else
                {
                    targetPos = mCurNode.objectPosition;//when you do not reach to your target point, just go back to your current point
                }
            }
            else
            {
                targetPos = mCurNode.objectPosition;//when there is no target just stay at your position
                Vector3 goal_3d_poind = new Vector3(goal_point[0], cRadius, goal_point[1]);
                if (flag_find_path && (goal_3d_poind - targetPos).magnitude < cRadius)
                    targetPos = goal_3d_poind; // new Vector3(goal_point[0], cRadius, goal_point[1]);
            }

            //if ((curPos - targetPos).magnitude > cRadius)
            //{
            //    Vector3 dir_to_target = (targetPos - curPos) / ((curPos - targetPos).magnitude + 0.000001f);
            //    Vector3 n_pos = curPos + (float)(path_length_onTree * cRadius) * dir_to_target;
            //    if ((curPos - n_pos).magnitude < cRadius / 2)
            //        path_length_onTree++;
            //    return curPos + (float)(path_length_onTree * cRadius) * dir_to_target;
            //}
            //else
            //{
            return targetPos;
            //}
        }

        public void restart_LazyRRT(Vector3 pos, Vector2 dir, float radius, Vector2 initial_goal)
        {
            numberOfChangedInPath = 0;//during the movement the path is going to change a couple of times, this will tell how many times it changed
            iteration_needed_for_firstPath = 0;//iteration needed for returning the first path
            cost_of_taken_path = 0;//iteration needed for returning the first path

            //tree variables

            current_root = 0;

            rateTowardGoal = 0.1f;
            //numberOfRRTSamples = ;
            path_length_onTree = 0;

            //informed RRT* var
            cur_cost_path = float.MaxValue;
            best_cost_path = float.MaxValue;
            cost_path_inside_notReachArea = 0;
            flag_calculate_path_cost = false;

            //obstacle variables (keep current pos to sense the movement of the obstacles)
            radius_check_obstacle_region = 10;

            //when it is necessary to change the cost_to_reach value of each node, is decided by the following variables
            numberOfUpdatingRoot = 0;
            numberOfUpdatingObstacles = 0;
            numberOfUpdatingGoals = 0;
            numberOfUpdatingStr = 0;

            //path variables
            flag_find_path = false;
            goal_index_inTree = -1;

            //For simulation
            cost_of_found_paths = new ArrayList();

            //queue 
            queue_random_samples = new m_queue_manager();
            queue_onTree_nodes = new m_queue_manager();

            /////obstacles' poses
            cur_pos_obstacles = new ArrayList();
            cur_pos_agent = pos;

            /////region manager
            mRegionManager = new regionManager(new Vector2(min_range_x, min_range_z), new Vector2(max_range_x, max_range_z), 2f);

            goal_point = new Vector2();
            TreeNodes = new ArrayList(5 * ((int)(max_range_x - min_range_x)) * ((int)(max_range_z - min_range_z)));

            cRadius = radius;
            radius_contact_region = (2 - (0 / 4f)) * cRadius;
            path_from_root = new ArrayList();

            set_goal_point(initial_goal[0], initial_goal[1]);
            add_node(pos, dir, radius, -1);//add first node
        }

        public Vector2 mPoint3Dto2D(Vector3 p)
        {
            return new Vector2(p[0], p[2]);
        }

        public Vector3 mPoint2Dto3D(Vector2 p)
        {
            return new Vector3(p[0], cRadius, p[1]);
        }
    }

    //number of forward prediction steps in the graph, computed from timeStep and planningHorizonSeconds
    int nSteps;
    [HideInInspector]
    //Something to hold the body id of the controlled ODE objects
    GameObject controlledBody; //targetGeom
    GameObject targetBody, targetBody1, targetBody2, targetBody3;

    /////////////////////////////////Lazy RRT params///////////////////////////////
    LazyRRT mRRT;


    int index_on_RRT;
    Vector3 targetPos;
    Vector3 move_to_pos;
    float max_sec = -1;
    ArrayList obstacles_pos;

    Vector3 init_agent_pos;
    bool not_done_yet = true;
    StreamWriter m_writer;
    ArrayList goal_poses;
    Vector3 current_goal;
    int next_goal_pos;

    float xd, yd;

    // Added params
    public Vector2 xBound = new Vector2(-15, 15);
    public Vector2 zBound = new Vector2(-15, 15);

    public static float ballRadius = 0.5f;
    public Vector3 goalPosition = new Vector3(0, ballRadius, 10);
    public float safeRadiusDObstacle = 2f;
    public float checkDObstacleDistance = 10f;
    Vector3 startPosition;
    long planTime;
    long arrivalTime;
    float plannedCost;
    int attempts;
    int nodeCount;
    bool success;
    float executedCost;
    bool planTimeObtained = false;
    bool finishedExperiment = false;
    bool collided = false;

    List<GameObject> dynamicObstaclesObjs;
    public LayerMask dynamicObstaclayer;

    int experimentCounter = 0;
    public int maxExperimentRuns = 50;

    public int maxIterationExperiment = 4500;
    public int iterationIncrement = 500;
    public int iterationExperiment = 50;

    public float gain = 2;
    public int experiment = 1;

    String expname = "_exp_";
    String scenename = "";


    List<ResultsManager> resultsList;
    MotionController motionController;
    ////////////////////////////////////End Params/////////////////////////////////
    // Use this for initialization
    String myFileGenerator()
    {
        StreamReader m_Reader;
        String mFile_path;
        String directory_name = "ResultsRRTStar\\";
        int num_file = 0;
        if (!Directory.Exists(directory_name))
            Directory.CreateDirectory(directory_name);
        if (File.Exists(directory_name + "number_file.txt"))
        {
            m_Reader = new StreamReader(directory_name + "number_file.txt");
            String str_num = m_Reader.ReadLine();
            num_file = ((int)str_num[0] - 48) + 1;
            mFile_path = directory_name + "results" + str_num + ".txt";
            m_Reader.Close();
        }
        else
        {
            mFile_path = directory_name + "results.txt";
        }
        StreamWriter m_writer_file = new StreamWriter(directory_name + "number_file.txt");
        m_writer_file.WriteLine(num_file);
        m_writer_file.Flush();
        m_writer_file.Close();
        return mFile_path;
    }
    List<GameObject> getAllObjectsInLayer(LayerMask layer)
    {
        UnityEngine.Object[] tempList = Resources.FindObjectsOfTypeAll(typeof(GameObject));
        List<GameObject> realList = new List<GameObject>();
        GameObject temp;

        foreach (UnityEngine.Object obj in tempList)
        {
            if (obj is GameObject)
            {
                temp = (GameObject)obj;
                if ((1 << temp.layer) == layer)
                    realList.Add((GameObject)obj);
            }
        }
        return realList;
    }
    void Start()
    {

#if _FIX_SEED_
        UnityEngine.Random.InitState(42); //44
#endif
        ballRadius = GetComponent<SphereCollider>().radius * Mathf.Max(transform.lossyScale.x, transform.lossyScale.y, transform.lossyScale.z);
        dynamicObstaclayer = LayerMask.GetMask("Dynamic Obstacles");
        dynamicObstaclesObjs = getAllObjectsInLayer(dynamicObstaclayer);
        //query the body id (integer) of the controlled object, as GetComponent and other unity methods
        //can't be called later from the optimizer background threads
        controlledBody = (GameObject.Find("Sphere") as GameObject);
        //        (GameObject.Find("Sphere") as GameObject).GetComponent<Collider>().enabled = false;//each agent should not collide with itself

        targetBody = (GameObject.Find("Target") as GameObject);
        targetBody1 = (GameObject.Find("Target1") as GameObject);
        targetBody2 = (GameObject.Find("Target2") as GameObject);
        targetBody3 = (GameObject.Find("Target3") as GameObject);
        (GameObject.Find("Target") as GameObject).GetComponent<Collider>().enabled = true;
        (GameObject.Find("Target1") as GameObject).GetComponent<Collider>().enabled = true;
        (GameObject.Find("Target2") as GameObject).GetComponent<Collider>().enabled = true;
        (GameObject.Find("Target3") as GameObject).GetComponent<Collider>().enabled = true;

        //obstacle velocity
        //targetBody.GetComponent<Rigidbody>().velocity = new Vector3(-1, 0, -1);
        //targetBody1.GetComponent<Rigidbody>().velocity = new Vector3(1, 0, 1);
        //targetBody2.GetComponent<Rigidbody>().velocity = new Vector3(1, 0, -1);
        //targetBody3.GetComponent<Rigidbody>().velocity = new Vector3(-1, 0, 0);

        nSteps = 32; //32

        ///////////////////////////set params of RRT/////////////////////////////////////
        String str_file_name = myFileGenerator();
        m_writer = new StreamWriter(str_file_name, true);
        Vector3 agent_pos = controlledBody.transform.position;
        init_agent_pos = controlledBody.transform.position;

        goal_poses = new ArrayList();
        goal_poses.Add(goalPosition);
        startPosition = this.transform.position;


        xd = ((Vector3)goal_poses[0]).x;
        yd = ((Vector3)goal_poses[0]).z;
        mRRT = new LazyRRT(new Vector3(agent_pos[0], ballRadius, agent_pos[2]), new Vector2(0, 1), ballRadius, new Vector2(xd, yd), iterationExperiment, checkDObstacleDistance, safeRadiusDObstacle, xBound, zBound);



        motionController = new MotionController(this, gain);
        resultsList = new List<ResultsManager>();
        index_on_RRT = 0;
        targetPos = new Vector3();
        move_to_pos = new Vector3();
        mRRT.setNextTargetPos(agent_pos, ref targetPos, ref index_on_RRT, 0);

        obstacles_pos = new ArrayList();
        obstacles_pos.Add(targetBody.transform.position);
        obstacles_pos.Add(targetBody1.transform.position);
        obstacles_pos.Add(targetBody2.transform.position);
        obstacles_pos.Add(targetBody3.transform.position);

        current_goal = new Vector3(xd, ballRadius, yd);
        next_goal_pos = 0;

        return;
    }

    int get_delta_miliSec(int cur_mili_sec)
    {
        int n_milisec = DateTime.Now.Millisecond;
        int delta_sec = n_milisec - cur_mili_sec;
        if (delta_sec < 0)
            delta_sec = 1000 - cur_mili_sec + n_milisec;
        return delta_sec;
    }

    void restart_from_beginning()
    {
        yd = init_agent_pos[2];
        xd = init_agent_pos[0];
        controlledBody.transform.position = init_agent_pos;
        mRRT.restart_LazyRRT(new Vector3(init_agent_pos[0], ballRadius, init_agent_pos[2]), new Vector2(0, 1), ballRadius, new Vector2(init_agent_pos[0], init_agent_pos[2]));
        index_on_RRT = 0;
        targetPos = new Vector3();
        move_to_pos = new Vector3();
        mRRT.setNextTargetPos(init_agent_pos, ref targetPos, ref index_on_RRT, 0);
        not_done_yet = true;
        return;
    }

    void DrawAsteriskDebug(Vector3 pt)
    {
        float minDist = 0.15f;
        for (int i = -1; i <= 1; i = i + 2)
        {
            for (int j = -1; j <= 1; j = j + 2)
            {
                for (int k = -1; k <= 1; k = k + 2)
                {
                    Vector3 addVect = new Vector3(i * minDist, j * minDist, k * minDist);
                    Debug.DrawLine(pt + addVect, pt, Color.red);
                }
            }
        }
    }

    void UpdateTargetWithMouse()
    {
        if (Input.GetMouseButtonDown(0))
        {
            RaycastHit hit;
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);

            if (Physics.Raycast(ray, out hit))
            {
                Transform objectHit = hit.transform;
                if (objectHit.name == "Plane")
                {
                    Vector2 goal_point = mRRT.mPoint3Dto2D(hit.point);
                    xd = goal_point[0];
                    yd = goal_point[1];
                }
                // Do something with the object that was hit by the raycast.
            }
        }
    }
    void OnCollisionEnter(Collision collision)
    {
        var tag = collision.gameObject.tag;
        if ((1 << collision.gameObject.layer) == LayerMask.GetMask("Dynamic Obstacles"))
        {
            collided = true;
        }

    }
    public void resetDObstacles()
    {
        for (int i = 0; i < dynamicObstaclesObjs.Count; i++)
        {
            dynamicObstaclesObjs[i].SendMessage("Reset");

        }
    }
    void Update()
    {
        UpdateTargetWithMouse();

        //RRT Part
        //DrawAsteriskDebug(mRRT.mPoint2Dto3D(new Vector2(xd, yd)));
        Vector3 m_agent_pos = controlledBody.transform.position;
        mRRT.mSimulation_numberOfNodes = 0;
        mRRT.FlagRewireTree = this.FlagRewireTree;
        mRRT.FlagInformedSampling = this.FlagInformedSampling;
        mRRT.FlagRewiringCircle = this.FlagRewiringCircle;
        mRRT.FlagUseColor = this.FlagUseColor;
        ////////////////////////////////Simulataion: Test 1////////////////////////////
        //bool flag_write_on_file = false;
        //if (!mRRT.isPathFound())
        //    mRRT.iteration_needed_for_firstPath++;
        //else
        //{
        //    if (next_goal_pos < goal_poses.Count)
        //    {
        //        current_goal = (Vector3)goal_poses[next_goal_pos];
        //        xd = current_goal[0];
        //        yd = current_goal[2];
        //        next_goal_pos++;
        //        flag_write_on_file = true;
        //    }

        //    if (flag_write_on_file)
        //    {
        //        if (next_goal_pos >= 1)
        //        {
        //            if (mRRT.iteration_needed_for_firstPath == 0)
        //                mRRT.iteration_needed_for_firstPath++;
        //            String str = mRRT.iteration_needed_for_firstPath.ToString() + " " + mRRT.goal_point[0].ToString() + " " + mRRT.goal_point[1].ToString();
        //            m_writer.WriteLine(str);
        //        }
        //        mRRT.iteration_needed_for_firstPath = 0;
        //    }
        //    else
        //    {
        //        if (not_done_yet)
        //        {
        //            if (mRRT.iteration_needed_for_firstPath == 0)
        //                mRRT.iteration_needed_for_firstPath++;
        //            String str = mRRT.iteration_needed_for_firstPath.ToString() + " " + mRRT.goal_point[0].ToString() + " " + mRRT.goal_point[1].ToString();
        //            m_writer.WriteLine(str);
        //            mRRT.iteration_needed_for_firstPath = 0;

        //            not_done_yet = false;
        //            m_writer.Flush();
        //            m_writer.Close();
        //        }
        //    }
        //}
        /////////////////////////////////////End Test1/////////////////////////////////////////////////

        //////////////////////////////Simulataion: Test 2////////////////////////////
        /// IMPORTANT
        bool flag_write_on_file = false;
        bool flag_change_goal = false;

        if (mRRT.finishedGrowingTree())
        {
            if (mRRT.isPathFound() & !planTimeObtained)
            {
                planTimeObtained = true;
                planTime = mRRT.getPlanTime();
                plannedCost = mRRT.cur_cost_path;
                attempts = mRRT.iterationsToFinish;
                nodeCount = mRRT.TreeNodes.Count;
                success = true;
                mRRT.iteration_needed_for_firstPath++;
            }
            if (!mRRT.isPathFound())
            {
                planTime = -1;
                plannedCost = -1;
                attempts = mRRT.iterationsToFinish;
                nodeCount = mRRT.TreeNodes.Count;
                success = false;
            }

        }



        if ((current_goal - m_agent_pos).magnitude < ballRadius || (mRRT.finishedGrowingTree() && success == false) || collided == true)
        {
            arrivalTime = mRRT.getPlanTime();
            executedCost = motionController.getExecutedCost();
            resultsList.Add(new ResultsManager { planTime = planTime, arrivalTime = arrivalTime, plannedCost = plannedCost, success = success, nodes = nodeCount, executedCost = executedCost, attempts = attempts, collision = collided });

            Debug.Log("planTime: " + planTime + ", arrivalTime: " + arrivalTime + ", plannedCost: " + plannedCost + ", executedCost: " + executedCost + ", success: " + success + ", collided: " + collided + ", attempts: " + attempts + ", nodes: " + nodeCount); //+ ", attempts: " + attempts + ", nodes: " + nodeCount); 
            Debug.Log("Previous run: #" + experimentCounter + ", Number of iterations: " + iterationExperiment);
            experimentCounter++;
            if (experimentCounter >= maxExperimentRuns)
            {
                iterationExperiment += iterationIncrement;
                experimentCounter = 0;
            }

            xd = ((Vector3)goal_poses[0]).x;
            yd = ((Vector3)goal_poses[0]).z;
            this.GetComponent<Rigidbody>().velocity = Vector3.zero;
            this.GetComponent<Rigidbody>().angularVelocity = Vector3.zero;
            this.transform.position = startPosition;
            m_agent_pos = controlledBody.transform.position;
            mRRT = new LazyRRT(new Vector3(startPosition[0], ballRadius, startPosition[2]), new Vector2(0, 1), ballRadius, new Vector2(xd, yd), iterationExperiment, checkDObstacleDistance, safeRadiusDObstacle, xBound, zBound);
            index_on_RRT = 0;
            targetPos = new Vector3();
            move_to_pos = new Vector3();
            mRRT.setNextTargetPos(init_agent_pos, ref targetPos, ref index_on_RRT, 0);
            not_done_yet = true;
            planTimeObtained = false;
            collided = false;
            motionController.clearCost();
            resetDObstacles();
        }


        if (iterationExperiment > maxIterationExperiment)
        {

            finishedExperiment = true;
        }

        if (finishedExperiment)
        {
            scenename = SceneManager.GetActiveScene().name;
            String datestr = DateTime.Now.ToString("yyyy-MM-dd HH.mm.ss");
            String filename1 = String.Concat("./Results/", scenename, expname, experiment.ToString(), "_", datestr, ".csv");
            using (var writer = new StreamWriter(filename1))
            using (var csv = new CsvWriter(writer, CultureInfo.InvariantCulture))
            {
                csv.WriteRecords(resultsList);
            }

            ExperimentManager experimentData = new ExperimentManager
            {
                maxExperimentRuns = maxExperimentRuns,
                maxIterationExperiment = maxIterationExperiment,
                iterationIncrement = iterationIncrement,
                iterationExperiment = iterationExperiment,
                gain = gain,
                experiment = experiment,
                scenename = scenename
            };
            List<ExperimentManager> experimentDataList = new List<ExperimentManager>();
            experimentDataList.Add(experimentData);

            String filename2 = String.Concat("./Results/", scenename, expname, experiment.ToString(), "_", datestr, "_param.csv");

            using (var writer = new StreamWriter(filename2))
            using (var csv = new CsvWriter(writer, CultureInfo.InvariantCulture))
            {
                csv.WriteRecords(experimentDataList);
            }

            EditorApplication.ExecuteMenuItem("Edit/Play");
        }

        if (flag_change_goal)
        {
            if (next_goal_pos < goal_poses.Count)
            {
                current_goal = (Vector3)goal_poses[next_goal_pos];
                xd = current_goal[0];
                yd = current_goal[2];
                next_goal_pos++;
                flag_write_on_file = true;
            }

            if (flag_write_on_file)
            {
                if (next_goal_pos >= 1)
                {
                    if (mRRT.iteration_needed_for_firstPath == 0)
                        mRRT.iteration_needed_for_firstPath++;
                    String str = mRRT.cost_of_taken_path.ToString() + " " + mRRT.iteration_needed_for_firstPath.ToString() + " " + mRRT.goal_point[0].ToString() + " " + mRRT.goal_point[1].ToString();
                    m_writer.WriteLine(str);
                }
                mRRT.iteration_needed_for_firstPath = 0;
            }
            else
            {
                if (not_done_yet)
                {
                    if (mRRT.iteration_needed_for_firstPath == 0)
                        mRRT.iteration_needed_for_firstPath++;
                    String str = mRRT.cost_of_taken_path.ToString() + " " + mRRT.iteration_needed_for_firstPath.ToString() + " " + mRRT.goal_point[0].ToString() + " " + mRRT.goal_point[1].ToString();
                    m_writer.WriteLine(str);
                    mRRT.iteration_needed_for_firstPath = 0;

                    not_done_yet = false;
                    m_writer.Flush();
                    m_writer.Close();
                }
            }
        }
        ///////////////////////////////////End Test 2/////////////////////////////////////////////////


        int my_max_time = 15;//you just have to put as low as possible to enable traversing on tree and to avoid lag in the game
        int cur_t = DateTime.Now.Millisecond;
        int delta_t = 0;
        /////update moving obstacle pos
        obstacles_pos[0] = targetBody.transform.position;//update moving obstacle pos
        obstacles_pos[1] = targetBody1.transform.position;//update moving obstacle pos
        obstacles_pos[2] = targetBody2.transform.position;//update moving obstacle pos
        obstacles_pos[3] = targetBody3.transform.position;//update moving obstacle pos
        //change goal point
        mRRT.change_goal_point(new Vector2(xd, yd));

        //////////////////////////take samples for RRT///////////////////////////////////
        bool flag_continue_time = true;
        for (int step = 0; step < nSteps && flag_continue_time; step++)
        {
            cur_t = DateTime.Now.Millisecond;
            mRRT.sampleSpace(true, flag_continue_time);
            delta_t += get_delta_miliSec(cur_t);
            if (delta_t > my_max_time)
                flag_continue_time = false;
        }

        mRRT.drawTree();
        //        cur_t = DateTime.Now.Millisecond;
        int next_index_onRRT = mRRT.find_best_path(index_on_RRT);
        //RRT: update target point of the controller

        bool moveOnlyWhenFinished;
        switch (experiment)
        {
            case 1:
                moveOnlyWhenFinished = mRRT.isPathFound() & mRRT.finishedGrowingTree();
                break;
            case 2:
                moveOnlyWhenFinished = true;
                break;
            case 3:
                moveOnlyWhenFinished = true;
                break;
            default:
                Debug.Log("No experiment defined, allowing robot to move!");
                moveOnlyWhenFinished = true;
                break;
        }


        if (FlagMoveAgent & moveOnlyWhenFinished)
            move_to_pos = mRRT.setNextTargetPos(m_agent_pos, ref targetPos, ref index_on_RRT, next_index_onRRT);
        mRRT.update_costs_basedOn_movingObstacles(obstacles_pos);
        if (FlagMoveAgent & moveOnlyWhenFinished)
        {
            motionController.control(move_to_pos, ballRadius / 2);
        }
        else
        {
            controlledBody.GetComponent<Rigidbody>().velocity = new Vector3(0, 0, 0);
            controlledBody.GetComponent<Rigidbody>().angularVelocity = new Vector3(0, 0, 0);
        }
        return;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        //Debug.Log("Tree node count: " + mRRT.TreeNodes.Count);

    }
    void OnApplicationQuit()
    {

        // The following voodoo is essential to avoid memory leaks in unity editor
#if UNITY_EDITOR
        UnityEditor.EditorUtility.UnloadUnusedAssetsImmediate();
#endif
        GC.Collect();
        m_writer.Close();
    }
}