
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEditor;
using Utils;
using UnityEngine.Assertions;
using System;


public class RTFMTPlanner
{
    int debugCounter = 0;

    //Game variables
    LayerMask obstaclesLayer;
    List<Transform> dynamicObstacles;
    float checkDObstacleDistance;
    public static float ballRadius;
    public static Vector3 offset3D;
    Vector3 xInit;

    MonoBehaviour robot;

    // Validator variables
    Vector2 xBound;
    Vector2 yBound;

    //List variables
    int sampleNumber;
    public List<Vector3> samplesList;
    List<Node> treeNodes;
    public Node rootNode;

    public List<Node> allNodes;
    public List<Node> unvisitedNodes;
    public List<Node> openNodes;
    public List<Node> closedNodes;

    public List<Node> blockedNodes;

    // Temporary variables
    List<Node> openNewNodes;
    List<Node.Neighbor> XNear;
    List<Node> closedToOpenNodes;
    Node zNode;

    // neighbor checking variables
    int dim;
    float rnScale = 1.1f;
    float rn;

    // Obstructed points
    public List<List<Node>> obstructedNodes; // List of obstructed nodes for each obstacle
    public List<Node> obstructedNodes1d; //Flattened version of obstructedNodes
    public float safeRadiusDObstacle;

    //Rewire nodes
    public List<Node> rewireRootList;
    List<Node> rewireRootListHistory;

    public List<Node> rewireLocalList;

    public Node goalNode;
    Node nearGoalNode;
    bool hasGoal = false;
    bool pathFound = false;
    bool fullPathFound = false;
    bool finished_ = false;
    bool success_ = false;
    float epsilonGoal;
    public List<Node> generatedPath;
    public List<Node> checkedPathCandidates;
    //float bestCost = 0;
    //float bestCostToGoal = float.MaxValue;

    Node oldRootNode = null;
    Node newRootNode = null;

    long start_ms;
    long prev_rewire_root_ms;
    long rewire_root_ms;


    int expandTreeRate = 32; //32


    const bool keep_searching = true; //used to keep searching path if no znode is found in case a dynamic obstacle is blocking it.
    public RTFMTPlanner(Vector2 xBound, Vector2 yBound, LayerMask obstaclesLayer, int dim, MonoBehaviour robot, List<Transform> dynamicObstacles, float ballRadius, float safeRadiusDObstacle, float checkDObstacleDistance)
    {

        this.xBound = xBound;
        this.yBound = yBound;
        this.dim = dim;
        this.obstaclesLayer = obstaclesLayer;
        this.dynamicObstacles = dynamicObstacles;

        this.robot = robot;

        RTFMTPlanner.ballRadius = ballRadius;
        RTFMTPlanner.offset3D =  new Vector3(0, ballRadius, 0);
        this.safeRadiusDObstacle = safeRadiusDObstacle;
        this.checkDObstacleDistance = checkDObstacleDistance;
}

    public void reset()
    {
        this.start_ms = DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;
        this.samplesList = new List<Vector3>();
        this.allNodes = new List<Node>();
        this.treeNodes = new List<Node>();
        this.openNodes = new List<Node>();
        this.unvisitedNodes = new List<Node>();
        this.closedNodes = new List<Node>();
        this.blockedNodes = new List<Node>();

        this.openNewNodes = new List<Node>();
        this.closedToOpenNodes = new List<Node>();
        this.XNear = new List<Node.Neighbor>();
        this.rewireRootList = new List<Node>();
        this.rewireRootListHistory = new List<Node>();
        this.rewireLocalList = new List<Node>();


        this.generatedPath = new List<Node>();
        this.checkedPathCandidates = new List<Node>();
        this.obstructedNodes = new List<List<Node>>(dynamicObstacles.Count);

        this.pathFound = false;
        this.oldRootNode = null;
        this.newRootNode = null;
        this.hasGoal = false;
        this.fullPathFound = false;
        this.finished_ = false;
        this.success_ = false;
    }
    public int getTreeNodes()
    {
        return this.treeNodes.Count;
    }
    public void init(Vector3 xInit, int sampleNumber)
    {

        this.xInit = xInit;

        this.reset();

        this.samplesList = generateUniformSamplesAndComputeRadius(sampleNumber);
        this.sampleNumber = sampleNumber;

        unvisitedNodes = pointsToNode(samplesList, NodeState.Unvisited);
        Node xInitNode = pointToNode(xInit, 0, NodeState.Undefined); //Once I add to open, I change the Node state
        addOpenNode(xInitNode);//   openNodes.Add(xInitNode);
        treeNodes.Add(xInitNode);
        rootNode = xInitNode;

        // Save all nodes in a single list
        allNodes = new List<Node>(unvisitedNodes);
        allNodes.Add(xInitNode);

        this.zNode = findZ(); //TODO: is there a way to find z in 1 iteration whenever I update the list?

#if _DEBUG_1_
		Debug.Log("num of elements in samplesList: " + samplesList.Count);
		Debug.Log("num of elements in unvisitedNodes: " + unvisitedNodes.Count);
		Debug.Log("num of elements in openNodes: " + openNodes.Count);
		Debug.Log("num of elements in closedNodes: " + closedNodes.Count);
		Debug.Log("num of elements in checkedPathCandidates: " + checkedPathCandidates.Count);
		Debug.Log("num of elements in path: " + generatedPath.Count);
#endif

#if _DEBUG_3_
        Debug.Log("num of elements in samplesList: " + samplesList.Count);
#endif
    }

    public long getPlanTime()
    {
        long cur_time_ms = DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;

        return (cur_time_ms - this.start_ms);
    }

    public void setGoal(Vector3 goalPosition) //TODO: This function must do much more than that!
    {
        //add goal to node list
        this.goalNode = new Node(goalPosition);
        addUnvisitedNode(goalNode);
        this.hasGoal = true;
        this.epsilonGoal = this.rn / 2;
        uncheckTree();
        this.resetPath();
    }
    public void setGoal(Node goalNode) //TODO: This function must do much more than that!
    {
        //add goal to node list
        this.goalNode = goalNode;
        this.hasGoal = true;
        this.epsilonGoal = this.rn / 2;
        uncheckTree();
        this.resetPath();
    }
    void uncheckTree()
    {
        for (int i = 0; i < checkedPathCandidates.Count; i++)
        {
            checkedPathCandidates[i].checkedPath = false;
        }
        checkedPathCandidates.Clear();
    }

    public void setRoot(Node newRoot)
    {
        if (newRoot.parent != null)
        {

            if (treeNodes.Contains(newRoot) && newRoot.parent.parent == null) // and parent is root
            {
                Node oldRootAndFather = rootNode;
                oldRootAndFather.children.Remove(newRoot);  // remove child (new root) of father (of root)
                newRoot.children.Add(oldRootAndFather); // Add old root  as child of new root

                oldRootAndFather.parent = newRoot;
                newRoot.parent = null;

                if (obstructedNodes1d.Contains(newRoot))
                {
                    newRoot.cost = float.MaxValue;
                }
                else
                {
                    newRoot.cost = 0;
                    blockedNodes.Remove(newRoot);
                }

                rootNode = newRoot;

                recalculateChildrenCost(rootNode);

                //if(rewireRootList.Count == 0)
                startRewireFromRoot();
            }
            //updatePath();

        }

    }


    public void update()
    {
        updateObstructedNodes();
        

        //rewireFromRoot1();
        
        //updateRewireFromRoot();
        for (int i = 0; i < expandTreeRate; i++)
        {
            rewireLocally();
            expandTree();
            rewireFromRoot2();
        }

        //searchPath();
        generatePath();
        //updateRoot();



#if _DEBUG_1_
		Debug.Log("Unvisited nodes count: " + unvisitedNodes.Count);
		Debug.Log("XNear Count: " + XNear.Count);
		Debug.Log("Open nodes ount: " + openNodes.Count);
		Debug.Log("Open new nodes count: " + openNewNodes.Count);
		Debug.Log("Closed nodes count: " + closedNodes.Count);
		Debug.Log("Tree nodes count: " + treeNodes.Count);
		Debug.Log("Blocked nodes count: " + blockedNodes.Count);
		Debug.Log("Total calls of neighbors is: " + Node.debugCountCallNeighbors);
		Debug.Log("num of elements in checkedPathCandidates: " + checkedPathCandidates.Count);
		Debug.Log("Num of elements in path: " + generatedPath.Count);
		debugNodes();
#endif
#if _DEBUG_2_
        if (zNode != null)
        {
            Drawer.debugDrawCircle(zNode.q, rn, Color.blue, 25);
        }
#endif
    }
    public void updateRoot()
    {
        oldRootNode = newRootNode;
        if (generatedPath.Count > 1) { 
            newRootNode = findNextNodeInPath(generatedPath);
        }
        if (newRootNode != null && (oldRootNode != newRootNode))
        {
            setRoot(newRootNode);
            generatedPath.Clear();
        }

    }
    public bool isPathFound()
    {
        return pathFound;
    }
    public bool hasPath()
    {
        return generatedPath.Count > 0;
    }
    public bool isFullPathFound()
    {
        return fullPathFound;
    }
    public bool finished()
    {
        return this.finished_;
    }
    public bool success()
    {
        return this.success_;
    }
    public List<Node> getPath()
    {
        return new List<Node>(generatedPath);
    }
    void generatePath()
    {
        pathFound = false; // only set to true when the path is updated
        fullPathFound = false;

        if (treeNodes.Contains(goalNode))
        {
            pathFound = generatePathBackward();
            fullPathFound = true;
            finished_ = true;
            success_ = true;
        }
        else
        {
            pathFound = generatePathForward();
        }
    }
    bool generatePathBackward()
    {
        Node nodeIter = goalNode;
        generatedPath.Clear();
        while (nodeIter != null)
        {
            generatedPath.Insert(0, nodeIter);
            nodeIter = nodeIter.parent;
        }
        return true;
    }
    Node findNextNodeInPath(List<Node> path)
    {
        Vector3 robotPos = robot.transform.position;
        Node nextNode = null;
        if (path != null)
        {
            int closestNodeIdx = -1;
            /*
            for (int i = 0; i < path.Count; i++)
            {
                float dist = (robot.transform.position - path[i].q).magnitude;
                if (dist < RTFMTPlanner.ballRadius)
                    closestNodeIdx = i;
            }*/

            float dist = (robot.transform.position - path[0].q).magnitude;
            if (dist < RTFMTPlanner.ballRadius)
                closestNodeIdx = 0;

            if (closestNodeIdx != -1)
            {
                int nextNodeIdx = Mathf.Min(closestNodeIdx + 1, path.Count - 1);
                nextNode = path[nextNodeIdx];
            }
        }
        return nextNode;
    }

    bool generatePathForward()
    {
        //generatedPath.Clear();
        List<Node> candidatePath = new List<Node>();
        Node cNode = rootNode;
        Node bestChildNode;
        bool flag_continue = true;
        bool pathFound = false;

        candidatePath.Add(cNode);

        if (cNode == goalNode)
        {
            flag_continue = false;
        }
        //While the current is not a leaf node
        while (flag_continue)
        {
            bestChildNode = findBestChild(cNode);
            if (bestChildNode != null)
            {
                cNode = bestChildNode;
                candidatePath.Add(cNode);

                if (bestChildNode == goalNode)
                    break;
            }
            else // if next test node is empty or it doesnt have children, stop
            {// this is a leaf
                cNode.checkedPath = true;
                checkedPathCandidates.Add(cNode);
                flag_continue = false;// there is no child to check
            }
        }
        //for every path I have to save its cost
        // The cost of the path has to be initiated as the coost of the root only
        if (generatedPath.Count == 0)
        {
            generatedPath.AddRange(candidatePath);
            pathFound = true;
        }
        else
        {
            if ((candidatePath[candidatePath.Count - 1].q - goalNode.q).magnitude < (generatedPath[generatedPath.Count - 1].q - goalNode.q).magnitude)
            {
                generatedPath.Clear();
                generatedPath.AddRange(candidatePath);
                pathFound = true;
            }
        }
        return pathFound;
    }
    void resetPath()
    {
        if (goalNode.parent == null)
        {
            pathFound = false;
        }
        generatedPath.Clear();
    }
    Node findBestChild(Node parent)
    {
        Node bestChildNode = null;
        float bestCostToGoal = float.MaxValue;
        for (int i = 0; i < parent.children.Count; i++)
        {
            Node childCandidate = parent.children[i];
            float costToGoal;
            if (childCandidate.checkedPath == false)// this list blocks a node
            {
                costToGoal = childCandidate.cost + (childCandidate.q - goalNode.q).magnitude;
            }
            else
            {
                costToGoal = float.MaxValue;
            }
            if (costToGoal < bestCostToGoal)
            {
                bestCostToGoal = costToGoal;
                bestChildNode = childCandidate;
            }
        }

        return bestChildNode;

    }

    void expandTree()
    {
        if (unvisitedNodes.Any() && !XNear.Any() && zNode != null) //If there are unvisited nodes and previous neighbors have been checked
        {
            // z has been updated. Calculate near samples to add to tree
            //XNear = near(unvisitedNodes, zNode);
            XNear = zNode.near(allNodes, rn, NodeState.Unvisited);

#if _DEBUG_1_
			Debug.Log("XNear updated. Count: " + XNear.Count);
#endif
        }
        if (XNear.Any()) // If there are nearSamples to test
        {
            Node.Neighbor x = ListExt.PopLast(XNear);
            List<Node.Neighbor> YNear = x.node.near(allNodes, rn, NodeState.Open);
            Node.Neighbor yMin;
            float yMinCost = argMinCost(x.node, YNear, out yMin, false); // Lazily Compute y with min cost to x
            if (yMin.node != null)
            {
                bool collisionFreeFixed = yMin.isCollisionFreeFObs; //isCollisionFree(yMin.q, x.node.q, agent.fixedObstaclesLayer);
                if (yMin.node.cost < float.MaxValue && collisionFreeFixed) //yMin.cost
                {
                    addNodeToTree(x.node, yMin.node, yMinCost);
                }
            }
            else
            {
                Debug.Log("Stop");

            }
#if _DEBUG_1_
			Debug.Log("YNearNodes updated. Count: " + YNear.Count);
#endif
#if _DEBUG_2_
            Drawer.debugDrawCircle(x.node.q, rn, Color.red, 25);
#endif
        }
        if (!XNear.Any() && zNode != null) // If nearSamples has emptied and we have a zNode
        {
            //Update open new nodes to open nodes
            addOpenNodes(openNewNodes); //openNodes.AddRange(openNewNodes);
            openNewNodes.Clear();

            // Close z node (remove from open as well)
            removeOpenNode(zNode); //   openNodes.Remove(zNode);
            addClosedNode(zNode);  //   closedNodes.Add(zNode);

            //If closed z node has unvisited neighbors and they are collision free from fixed obstacles
            List<Node.Neighbor> zNodeNear = zNode.near(allNodes, rn, NodeState.Unvisited);
            for (int i = 0; i < zNodeNear.Count; i++)
            {
                if (zNodeNear[i].isCollisionFreeFObs) // TODO: AND IF OBSTACLE MOVED
                {
                    closedToOpenNodes.Add(zNode);
                    break;
                }
            }
            // Find new z
            zNode = findZ();
        }
        if (zNode == null)
        {
            if (keep_searching && closedToOpenNodes.Count > 0) // keep searching for z nodes just in case a dynamic obstacle frees up unvisited nodes.
            {
                for (int i = 0; i < closedToOpenNodes.Count; i++)
                {
                    removeClosedNode(closedToOpenNodes[i]);
                    addOpenNode(closedToOpenNodes[i]);
                }
                closedToOpenNodes.Clear();
                // Find new z
                this.zNode = findZ();
            }
            else
            {
                if(!treeNodes.Contains(goalNode))
                     this.finished_ = true;
            }
        }
    }

    // Problem: Once a node is cleared, it stays in the blockednodes list (Fixed?)
    void updateObstructedNodes()
    {
        // Find nodes that may have been obstructed or clear
        List<Node> nodesToBlock;
        List<Node> nodesToUnblock;
        findObstructedNodes(out nodesToBlock, out nodesToUnblock);

#if _DEBUG_1_
		Debug.Log("Number of nodes to block: " + nodesToBlock.Count);
		Debug.Log("Number of nodes to unblock: " + nodesToUnblock.Count);
#endif
        //Now that we have the nodes, we just need to block or unblock them.
        //Blocking nodes
        for (int i = 0; i < nodesToBlock.Count; i++)
        {
            Node nodeToBlock = nodesToBlock[i];

            if (!blockedNodes.Contains(nodeToBlock))
            {
                nodeToBlock.cost = float.MaxValue;
                blockedNodes.Add(nodeToBlock);
                recalculateChildrenCost(nodeToBlock); // this method will also block children
            }
        }
        //unblocking nodes
        for (int i = 0; i < nodesToUnblock.Count; i++)
        {
            Node nodeToUnblock = nodesToUnblock[i];
            if (nodeToUnblock.parent == null) // This is a parent
            {
                nodeToUnblock.cost = 0;
                blockedNodes.Remove(nodeToUnblock);
                recalculateChildrenCost(nodeToUnblock); // this method will also unblock children
            }
            else if (nodeToUnblock.parent.cost < float.MaxValue)
            {
                nodeToUnblock.cost = nodeToUnblock.parent.cost + Node.costFun(nodeToUnblock.parent, nodeToUnblock);
                blockedNodes.Remove(nodeToUnblock);
                recalculateChildrenCost(nodeToUnblock); // this method will also unblock children
            }
        }
    }

    // RecalculateCost recalculates the cost for all childrens of node and:
    // if prev cost was inf and now it isnt -> unblock node
    // if prev cost wasn't inf and now it is -> block node
    public void recalculateChildrenCost(Node node)
    {
        Queue<Node> updateQueue = new Queue<Node>();

        //Add all childrens of input node
        for (int i = 0; i < node.children.Count; i++)
        {
            updateQueue.Enqueue(node.children[i]);
        }

        while (updateQueue.Count != 0) // while there are nodes in queue to update
        {
            Node nodeToUpdate = updateQueue.Peek(); //Get node to update
            if (!obstructedNodes1d.Contains(nodeToUpdate)) // If node is not obstructed, free it. Otherwise, remove it without adding its children
            {
                float prevCost = nodeToUpdate.cost;
                nodeToUpdate.cost = nodeToUpdate.parent.cost + Node.costFun(nodeToUpdate.parent, nodeToUpdate); //update it

                updateQueue.Dequeue(); // Remove from queue

                if (prevCost < float.MaxValue && nodeToUpdate.cost == float.MaxValue)
                {
                    if (!blockedNodes.Contains(nodeToUpdate))
                    {
                        blockedNodes.Add(nodeToUpdate);
                    }
                }
                else if (prevCost == float.MaxValue && nodeToUpdate.cost < float.MaxValue)
                {
                    blockedNodes.Remove(nodeToUpdate);
                }

                //Add childrens
                for (int i = 0; i < nodeToUpdate.children.Count; i++)
                {
                    updateQueue.Enqueue(nodeToUpdate.children[i]);
                }
            }
            else
            {
                //nodeToUpdate.cost = float.MaxValue; // not necessary
                updateQueue.Dequeue(); // Remove from queue
            }
        }
    }

    void rewireLocally()
    {
        if (rewireLocalList.Count == 0)
        {
            rewireLocalList = new List<Node>(blockedNodes);
        }
        else
        {
            Node xBlockedChild = rewireLocalList.PopFirst();
            if(xBlockedChild != null)
            {

            
            if (!obstructedNodes1d.Contains(xBlockedChild))
            {
                List<Node.Neighbor> YNear = xBlockedChild.near(allNodes, rn, NodeState.Open, NodeState.Closed);
                Node.Neighbor yMinParent;
                float yMinCost = argMinCost(xBlockedChild, YNear, out yMinParent, true); // Compute y with min cost to x
                if (yMinParent.node != null)
                {
                    bool collisionFreeFixed = yMinParent.isCollisionFreeFObs;//isCollisionFree(yMinParent.q, xBlockedChild.q, agent.fixedObstaclesLayer);
                    if (yMinParent.node.cost < float.MaxValue && collisionFreeFixed)
                    {
                            if(xBlockedChild != null && xBlockedChild.parent != null && xBlockedChild.parent.children.Count > 0)
                            {
                                xBlockedChild.parent.children.Remove(xBlockedChild); // Remove xTestChild from previous parent
                                yMinParent.node.addChildUpdateParent(xBlockedChild, yMinCost); // add child to new parent
                                blockedNodes.Remove(xBlockedChild);
                                recalculateChildrenCost(xBlockedChild);
                            }
                        
                    }
                }
            }
            }
        }
    }
    // This function loops through all nodes radially and looks for better children for each node

    void clearListAddRoot()
    {
        rewireRootList.Add(rootNode);
        rewireRootListHistory.Clear();
        this.prev_rewire_root_ms = this.rewire_root_ms;
        this.rewire_root_ms = DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;
        //Debug.Log("Time to rewire from Root: " + (rewire_root_ms - prev_rewire_root_ms));
    }
    void rewireFromRoot1()
    {
        if (rewireRootList.Count == 0)
        {
            clearListAddRoot();
        }
        Node xTestParent = rewireRootList.PopFirst();
        List<Node.Neighbor> XTestChildren = xTestParent.near(allNodes, rn, NodeState.Open, NodeState.Closed);

        for (int i = 0; i < XTestChildren.Count; i++)
        {
            Node xTestChild = XTestChildren[i].node;
            float cOld = xTestChild.cost;
            float cNew = xTestParent.cost + Node.costFun(xTestParent, xTestChild);
            bool collisionFreeDynamic = !obstructedNodes1d.Contains(xTestChild);
            bool collisionFreeFixed = XTestChildren[i].isCollisionFreeFObs;//isCollisionFree(xTestParent.q, xTestChild.q, agent.fixedObstaclesLayer); //TODO: We don't need this line anymore
            if (cNew < cOld && collisionFreeDynamic && collisionFreeFixed)
            {
                //Change parent and update children
                xTestChild.parent.children.Remove(xTestChild); // Remove xTestChild from previous parent
                xTestParent.addChildUpdateParent(xTestChild, cNew); // add child to new parent
                blockedNodes.Remove(xTestChild);
                //recalculateChildrenCost(xTestChild); 
            }

            if (!rewireRootListHistory.Contains(xTestChild))
            {
                rewireRootList.Add(xTestChild);
                rewireRootListHistory.Add(xTestChild);
            }
        }
    }

    // This function loops through all nodes radially and looks for better parent for each node
    void rewireFromRoot2()
    {
        if (rewireRootList.Count == 0)
        {
            clearListAddRoot();
        }
        Node xTestChild = rewireRootList.PopFirst();
        List<Node.Neighbor> XTestParents = xTestChild.near(allNodes, rn, NodeState.Open, NodeState.Closed);

        for (int i = 0; i < XTestParents.Count; i++)
        {
            Node xTestParent = XTestParents[i].node;
            float cOld = xTestChild.cost;
            float cNew = xTestParent.cost + Node.costFun(xTestParent, xTestChild);
            bool collisionFreeDynamic = !obstructedNodes1d.Contains(xTestChild); //xTestChild?
            bool collisionFreeFixed = XTestParents[i].isCollisionFreeFObs;//isCollisionFree(xTestParent.q, xTestChild.q, agent.fixedObstaclesLayer); //TODO: We don't need this line anymore
            if (cNew < cOld && collisionFreeDynamic && collisionFreeFixed)
            {
                //Change parent and update children
                xTestChild.parent.children.Remove(xTestChild); // Remove xTestChild from previous parent
                xTestParent.addChildUpdateParent(xTestChild, cNew); // add child to new parent
                blockedNodes.Remove(xTestChild);
                recalculateChildrenCost(xTestChild);
            }

            if (!rewireRootListHistory.Contains(xTestParent))
            {
                rewireRootList.Add(xTestParent);
                rewireRootListHistory.Add(xTestParent);
            }
        }
    }

    void startRewireFromRoot()
    {
        rewireRootList.Clear();
        clearListAddRoot();
    }
    // This function improves from rewireFromRoot2 by using ArgMinCost
    void updateRewireFromRoot()
    {
        if (rewireRootList.Count != 0)
        {
            Node xTestChild = rewireRootList.PopFirst();
            List<Node.Neighbor> YNearParents = xTestChild.near(allNodes, rn, NodeState.Open, NodeState.Closed);
            if (!obstructedNodes1d.Contains(xTestChild))
            {
                Node.Neighbor yMinParent;
                float yMinCost = argMinCost(xTestChild, YNearParents, out yMinParent, true); // Compute y with min cost to x
                if (yMinParent.node != null)
                {
                    bool collisionFreeFixed = yMinParent.isCollisionFreeFObs;//isCollisionFree(yMinParent.q, xTestChild.q, agent.fixedObstaclesLayer);
                    if (yMinParent.node.cost < float.MaxValue && xTestChild.cost > yMinCost && collisionFreeFixed) // xTestChild cost might be smaller than yMin if node is already on tree
                    {
                        xTestChild.parent.children.Remove(xTestChild); // Remove xTestChild from previous parent
                        yMinParent.node.addChildUpdateParent(xTestChild, yMinCost); // add child to new parent
                        blockedNodes.Remove(xTestChild);
                        recalculateChildrenCost(xTestChild);
                    }
                }
            }
            for (int i = 0; i < YNearParents.Count; i++)
            {
                if (!rewireRootListHistory.Contains(YNearParents[i].node))
                {
                    rewireRootList.Add(YNearParents[i].node);
                    rewireRootListHistory.Add(YNearParents[i].node);
                }
            }

            //updatePath();
        }
    }

    void findObstructedNodes(out List<Node> nodesToBlock, out List<Node> nodesToUnblock)
    {
        // This function finds the current obstructed nodes and compares with the previous one.
        // TODO: Instead of just clearing obstructedNodes, first check if an obstacle has moved a certain distance.
        // Then, only update the list for that obstacle to save processing time

        List<Node> prevObstructedNodes1d = obstructedNodes.SelectMany(list => list).ToList();
        obstructedNodes.Clear();
        for (int i = 0; i < dynamicObstacles.Count; i++)
        {
            if((dynamicObstacles[i].position - robot.transform.position).magnitude < checkDObstacleDistance)
            {
                List<Node> nodes = nodesInRadius(dynamicObstacles[i].position, safeRadiusDObstacle);
                if (nodes != null)
                {
                    obstructedNodes.Add(nodes);
                }
            }
        }
        obstructedNodes1d = obstructedNodes.SelectMany(list => list).ToList();

        nodesToBlock = new List<Node>(obstructedNodes1d.Except(prevObstructedNodes1d)); // new blocked nodes
        nodesToUnblock = new List<Node>(prevObstructedNodes1d.Except(obstructedNodes1d)); // new blocked nodes
    }

    List<Node> nodesInRadius(Vector3 pos, float radius)
    {
        List<Node> nodesInRadius = new List<Node>();
        for (int i = 0; i < treeNodes.Count; i++)
        {
            Node node = treeNodes[i];
            if ((node.q - pos).magnitude < radius)
            {
                nodesInRadius.Add(node);
            }
        }
        return nodesInRadius;
    }

    bool isCollisionFree(Vector3 p1, Vector3 p2, LayerMask layer)
    {
        //float cRadius = 0.5f;
        RaycastHit collisionInfo;

        Vector3 dir = p2 - p1;
        //bool result = !Physics.Raycast(p1, dir, out collisionInfo, dir.magnitude, layer);
        //bool result = !Physics.Raycast(p1, dir, out collisionInfo, dir.magnitude, layer);
        bool result = !Physics.SphereCast(p1, RTFMTPlanner.ballRadius, dir, out collisionInfo, dir.magnitude, layer);

#if _DEBUG_1_
		Debug.DrawRay(p1, dir, Color.red);
#endif
        return result;
    }

    void addNodeToTree(Node child, Node parent, float cost)
    {
        removeUnvisitedNode(child); //unvisitedNodes.Remove(child);
        parent.addChildUpdateParent(child, cost);
        treeNodes.Add(child);
        openNewNodes.Add(child);
        child.uncheckAncestrals(checkedPathCandidates);

    }

    float argMinCost(Node x, List<Node.Neighbor> YNear, out Node.Neighbor yMin, bool checkObstacles)
    {
        float yMinCost = float.MaxValue;
        yMin = default(Node.Neighbor);
        for (int i = 0; i < YNear.Count; i++)
        {
            if (checkObstacles && !YNear[i].isCollisionFreeFObs)
            {
                continue; // This condition ignores nodes that are blocked by an obstacle
            }
            Node yNode = YNear[i].node;
            float totalCost = yNode.cost + Node.costFun(yNode, x);

            if (totalCost <= yMinCost)
            {
                yMinCost = totalCost;
                yMin = YNear[i];
            }
        }
        return yMinCost;
    }

    Node findZ()
    {
        Node z = null;
        float[] costs = openNodes.Select(x => x.cost).ToArray();
        if (costs.Length > 0)
        {
            int index = findMinIndex(costs);
#if _DEBUG_1_
			Debug.Log("Min Index: " + index + " Cost: " + openNodes[index].cost);
#endif
            z = openNodes[index];
        }
        return z;
    }

    int findMinIndex(float[] list)
    {
        int pos = 0;
        for (int i = 0; i < list.Length; i++)
        {
            if (list[i] < list[pos]) { pos = i; }
        }
        return pos;
    }

    public List<Vector3> generateUniformSamplesAndComputeRadius(int numSamples)
    {
        List<Vector3> samplesList_ = new List<Vector3>(numSamples);
        int trials = 0;
        int success = 0;
        while (samplesList_.Count < numSamples)
        {
            Vector3 sampledVector = samplePoint();
            bool collision = Physics.CheckSphere(sampledVector, RTFMTPlanner.ballRadius, obstaclesLayer);
            if (!collision)
            {
                samplesList_.Add(sampledVector);
                success++;
            }
            trials++;
        }
        computeRadius(success, trials, numSamples);
        return samplesList_;
    }

    // 
    void computeRadius(int success, int trials, int numSampples)
    {
        float zeta = unitBallVolume(dim);
        float mu = freeVolume(success, trials);

        float gamma = 2 * Mathf.Pow((1 + 1 / (float)dim), (1 / (float)dim)) * Mathf.Pow((mu / zeta), (1 / (float)dim)); // (1 + 1 / d) is from PRM not FMT, which is 1 / d only

        rn = rnScale * gamma * Mathf.Pow((Mathf.Log10(numSampples) / (float)numSampples), (1 / (float)dim));

#if _DEBUG_1_
		Debug.Log("The neighbor radius is: " + rn);
#endif
    }

    // freeVolume computes the estimate of the obstacle-free part of the environment
    // using the upper confidence bound.
    // It is safer to mistake the volume bigger than it actually is.
    float freeVolume(int success, int trials)
    {
        float area = totalArea();
        Vector2 CI = Prob.binomialInterval(success, trials);

        float freeVol = CI[1] * area;
#if _DEBUG_1_
		Debug.Log("The free volume is: " + freeVol);
#endif
        return freeVol;
    }

    public float unitBallVolume(int dim)
    {
        Assert.IsTrue(dim >= 0);
        if (dim == 0)
        {
            return 1;
        }
        else if (dim == 1)
        {
            return 2;
        }
        else
        {
            return 2 * Mathf.PI / dim * unitBallVolume(dim - 2);

        }
    }

    float totalArea()
    {
        float area;
        float diffx = this.xBound[1] - this.xBound[0];
        float diffy = this.yBound[1] - this.yBound[0];
        area = diffx * diffy;
        return area;
    }

    public Vector3 samplePoint()
    {
        Vector3 sampledVector = new Vector3(UnityEngine.Random.Range(xBound[0], xBound[1]), RTFMTPlanner.ballRadius, UnityEngine.Random.Range(yBound[0], yBound[1]));
        return sampledVector;
    }
    public Vector3 sampleValidPoint()
    {
        Vector3 sampledVector = Vector3.zero;
        bool collision = true;
        while (collision)
        {
            sampledVector = samplePoint();
            collision = Physics.CheckSphere(sampledVector, RTFMTPlanner.ballRadius, obstaclesLayer);
        }
            //TODO: check fixed and moving obstacle
        return sampledVector;
    }

    Node pointToNode(Vector3 point, float cost, NodeState state)
    {
        return new Node(point, cost, state);
    }

    List<Node> pointsToNode(List<Vector3> points, NodeState state)
    {
        List<Node> list = new List<Node>();
        foreach (var sample in points)
        {
            list.Add(new Node(sample, -1, state));
        }
        return list;
    }

    //addUnvisitedNode also adds the node to allNodes and also updates the neighbors
    public void addUnvisitedNode(Node nodeToAdd)
    {
        allNodes.Add(nodeToAdd);
        unvisitedNodes.Add(nodeToAdd);
        nodeToAdd.state = NodeState.Unvisited;
        nodeToAdd.computeAllNeighbors(allNodes, rn);
        //Get all neighbor nodes and add nodeToAdd as their neighbor as well
        for (int i = 0; i < nodeToAdd.neighbors.Count; i++)
        {
            Node.Neighbor neighborToNodeToAdd = nodeToAdd.neighbors[i];
            Node.Neighbor nodeToAddasNeighbor = new Node.Neighbor();
            nodeToAddasNeighbor.node = nodeToAdd;
            nodeToAddasNeighbor.isCollisionFreeFObs = neighborToNodeToAdd.isCollisionFreeFObs; // If a is collision free to b, then b is also collision free to a
            neighborToNodeToAdd.node.neighbors.Add(nodeToAddasNeighbor);
        }
    }
    public void removeUnvisitedNode(Node node)
    {
        unvisitedNodes.Remove(node);
        node.state = NodeState.Undefined;
    }
    public void addOpenNode(Node node)
    {
        openNodes.Add(node);
        node.state = NodeState.Open;
    }
    public void addOpenNodes(List<Node> nodes)
    {
        for (int i = 0; i < nodes.Count; i++)
        {
            openNodes.Add(nodes[i]);
            nodes[i].state = NodeState.Open;
        }
    }
    public void removeOpenNode(Node node)
    {
        openNodes.Remove(node);
        node.state = NodeState.Undefined;
    }
    public void addClosedNode(Node node)
    {
        closedNodes.Add(node);
        node.state = NodeState.Closed;
    }
    public void removeClosedNode(Node node)
    {
        closedNodes.Remove(node);
        node.state = NodeState.Undefined;
    }

    public Node nearestNode(Vector3 point)
    {
        Node nearestNode = new Node();
        float minDist = float.MaxValue;
        for (int i = 0; i < treeNodes.Count; i++)
        {
            float dist = (point - treeNodes[i].q).magnitude;
            if (dist < minDist)
            {
                minDist = dist;
                nearestNode = treeNodes[i];
            }
        }
        return nearestNode;
    }

    // Drawing functions
    public void drawTree()
    {
        for (int i = 0; i < treeNodes.Count; i++)
        {
            Node curNode = treeNodes[i];
            if (curNode.parent != null)
            {
                Debug.DrawLine(curNode.parent.q, curNode.q, Color.black);
            }
        }
    }
    public void drawPath()
    {
        for (int i = 0; i < generatedPath.Count - 1; i++)
        {
            Node curNode = generatedPath[i];
            Node nextNode = generatedPath[i + 1];
            //Debug.DrawLine(curNode.q + RTFMT.offset3D, nextNode.q + RTFMT.offset3D, Color.red);

            var p1 = curNode.q + RTFMTPlanner.offset3D;
            var p2 = nextNode.q + RTFMTPlanner.offset3D;
            var thickness = 4;
            Handles.DrawBezier(p1, p2, p1, p2, Color.red, null, thickness);
        }
    }

    void debugNodes()
    {
        int openCount = 0;
        int closedCount = 0;
        int unvisitedCount = 0;
        int undefinedCount = 0;

        if (allNodes.Count > 0)
        {
            for (int i = 0; i < allNodes.Count; i++)
            {
                Node node = allNodes[i];
                switch (node.state)
                {
                    case NodeState.Closed:
                        closedCount++;
                        break;
                    case NodeState.Open:
                        openCount++;
                        break;
                    case NodeState.Unvisited:
                        unvisitedCount++;
                        break;
                    default:
                        undefinedCount++;
                        break;
                }

            }
            Debug.Log("Debug node states");
            Debug.Log("Unvisited Count: " + unvisitedCount);
            Debug.Log("Open Count: " + openCount);
            Debug.Log("Closed Count: " + closedCount);
            Debug.Log("Undefined Count: " + undefinedCount);

        }
    }
}
public enum NodeState
{
    Undefined,
    Open,
    Unvisited,
    Closed
}

public class Node
{
    public struct Neighbor
    {
        public Node node;
        public bool isCollisionFreeFObs;
    }

    public static int debugCountCallNeighbors = 0;
    public Vector3 q;
    public Node parent;
    public List<Node> children;
    public float cost;
    public List<Neighbor> neighbors;
    //public List<bool> cFreeFixObsNeighbors; // holds true if neighbor of same index is collision free
    public bool calculatedneighbors = false;
    public bool checkedPath = false; // Used to generate path


    public NodeState state;

    //specific to Unity 3D
    public static LayerMask fixedObstaclesLayer = LayerMask.GetMask("Obstacles");

    public Node()
    {
        this.cost = -1;
        children = new List<Node>();
        neighbors = new List<Neighbor>();
    }
    public Node(Vector3 q)
    {
        this.q = q;
        this.cost = -1;
        children = new List<Node>();
        neighbors = new List<Neighbor>();
    }
    public Node(Vector3 q, float cost)
    {
        this.q = q;
        this.cost = cost;
        children = new List<Node>();
        neighbors = new List<Neighbor>();
    }
    public Node(Vector3 q, float cost, NodeState state)
    {
        this.q = q;
        this.cost = cost;
        children = new List<Node>();
        neighbors = new List<Neighbor>();
        this.state = state;
    }
    public Node(Vector3 q, Node parent)
    {
        this.q = q;
        this.parent = parent;
        this.cost = -1;
        neighbors = new List<Neighbor>();
        children = new List<Node>();
    }
    public Node(Vector3 q, Node parent, float cost)
    {
        this.q = q;
        this.parent = parent;
        this.cost = cost;
        neighbors = new List<Neighbor>();
        children = new List<Node>();
    }

    public void addChildUpdateParent(Node child, float cost)
    {
        this.children.Add(child);
        child.parent = this;
        child.cost = cost;
    }

    public int childrenCount()
    {
        return this.children.Count;
    }

    // RecalculateCost recalculates the cost for all childrens of node and:
    // if prev cost was inf and now it isnt -> unblock node
    // if prev cost wasn't inf and now it is -> block node
    public void recalculateChildrenCost()
    {
        Queue<Node> updateQueue = new Queue<Node>();

        //Add all childrens of input node
        for (int i = 0; i < this.children.Count; i++)
        {
            updateQueue.Enqueue(this.children[i]);
        }

        while (updateQueue.Count != 0) // while there are nodes in queue to update
        {
            Node nodeToUpdate = updateQueue.Peek(); //Get node to update
            float prevCost = nodeToUpdate.cost;
            nodeToUpdate.cost = nodeToUpdate.parent.cost + Node.costFun(nodeToUpdate.parent, nodeToUpdate); //update it
            updateQueue.Dequeue(); // Remove from queue

            //Add childrens
            for (int i = 0; i < nodeToUpdate.children.Count; i++)
            {
                updateQueue.Enqueue(nodeToUpdate.children[i]);
            }
        }

    }
    List<Neighbor> getNeighbors(NodeState state)
    {
        List<Neighbor> result = new List<Neighbor>();

        if (calculatedneighbors) // if neighbors not calculated
        {
            for (int i = 0; i < neighbors.Count; i++)
            {
                if (neighbors[i].node.state == state)
                {
                    result.Add(neighbors[i]);
                }
            }
        }
        return result;
    }
    List<Neighbor> getNeighbors(NodeState state1, NodeState state2)
    {
        List<Neighbor> result = new List<Neighbor>();

        if (calculatedneighbors) // if neighbors not calculated
        {
            for (int i = 0; i < neighbors.Count; i++)
            {
                if ((neighbors[i].node.state == state1) || (neighbors[i].node.state == state2))
                {
                    result.Add(neighbors[i]);
                }
            }
        }
        return result;
    }

    public void computeAllNeighbors(List<Node> list, float radius)
    {
        Node nodeInList;
        debugCountCallNeighbors++;
        neighbors.Clear();
        calculatedneighbors = false;
        if (this.q != null) // if node is well defined
        {
            for (int i = 0; i < list.Count; i++)
            {
                nodeInList = list[i];
                float dist = (nodeInList.q - this.q).magnitude;
                if (dist < radius && nodeInList != this)
                {
                    Neighbor neighbor = new Neighbor();
                    neighbor.node = nodeInList;
                    bool collisionResult = this.isCollisionFree(nodeInList, fixedObstaclesLayer, RTFMTPlanner.ballRadius);//RTFMT.ballRadius);
                    neighbor.isCollisionFreeFObs = collisionResult;
                    neighbors.Add(neighbor);
                }
            }

            calculatedneighbors = true;
        }
        return;
    }
    public bool isCollisionFree(Node node, LayerMask layer, float ballRadius)
    {
        //float cRadius = 0.5f;
        RaycastHit collisionInfo;

        Vector3 dir = node.q - this.q;
        //bool result = !Physics.Raycast(p1, dir, out collisionInfo, dir.magnitude, layer);
        //bool result = !Physics.Raycast(p1, dir, out collisionInfo, dir.magnitude, layer);
        bool result = !Physics.SphereCast(this.q, ballRadius, dir, out collisionInfo, dir.magnitude, layer);

#if _DEBUG_1_
		Debug.DrawRay(this.q, dir, Color.red);
#endif
        return result;
    }

    public List<Neighbor> near(List<Node> list, float radius, NodeState state1)
    {
        // always pass all nodes possible, and select the intersection in "state"
        List<Neighbor> nearNodes = null;
        if (!calculatedneighbors) // if neighbors not calculated
        {
            computeAllNeighbors(list, radius);
            nearNodes = getNeighbors(state1);
        }
        else
        {
            nearNodes = getNeighbors(state1);
        }
        return nearNodes;
    }
    public List<Neighbor> near(List<Node> list, float radius, NodeState state1, NodeState state2)
    {
        // always pass all nodes possible, and select the intersection in "state"
        List<Neighbor> nearNodes = null;
        if (!calculatedneighbors) // if neighbors not calculated
        {
            computeAllNeighbors(list, radius);
            nearNodes = getNeighbors(state1, state2);
        }
        else
        {
            nearNodes = getNeighbors(state1, state2);
        }
        return nearNodes;
    }

    public void uncheckAncestrals(List<Node> checkedPathCandidates)
    {
        Node node = this;
        if (node.checkedPath == false)
        {
            node = node.parent;
            while (node != null)
            {
                node.checkedPath = false;
                checkedPathCandidates.Remove(node);
                node = node.parent;
            }
        }

    }

    public float costFun(Node node2)
    {
        return (this.q - node2.q).magnitude;
    }

    public static float costFun(Node node1, Node node2)
    {
        return (node1.q - node2.q).magnitude;
    }

    public static bool isAllChecked(List<Node> nodes)
    {
        bool result = true;
        for (int i = 0; i < nodes.Count; i++)
        {
            if (nodes[i].checkedPath == false)
            {
                result = false;
            }
        }
        return result;
    }

}
public enum Sampler
{
    Uniform,
    PoissonDisc
}
